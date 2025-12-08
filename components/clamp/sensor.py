import logging
import esphome.codegen as cg
from esphome.components import sensor, voltage_sampler, udp
from esphome.components.esp32 import get_esp32_variant
from esphome.components.esp32 import add_idf_sdkconfig_option
import esphome.config_validation as cv
from esphome.const import (
    CONF_ATTENUATION,
    CONF_ID,
    CONF_NUMBER,
    CONF_PIN,
    CONF_RAW,
    CONF_WIFI,
    CONF_FREQUENCY,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
)
from esphome.core import CORE
import esphome.final_validate as fv

from . import (
    ATTENUATION_MODES,
    ESP32_VARIANT_ADC1_PIN_TO_CHANNEL,
    ESP32_VARIANT_ADC2_PIN_TO_CHANNEL,
    SAMPLING_MODES,
    adc_ns,
    validate_adc_pin,
)

_LOGGER = logging.getLogger(__name__)

AUTO_LOAD = ["voltage_sampler","sensor"]

CONF_SAMPLES = "samples"
CONF_SAMPLING_MODE = "sampling_mode"
CONF_PIN_A1 = "pinA1"
CONF_PIN_A2 = "pinA2"
CONF_PIN_A3 = "pinA3"
CONF_PIN_V = "pinV"
CONF_UDP = "udp"

_attenuation = cv.enum(ATTENUATION_MODES, lower=True)
_sampling_mode = cv.enum(SAMPLING_MODES, lower=True)


def validate_config(config):
    if config[CONF_RAW] and config.get(CONF_ATTENUATION, None) == "auto":
        raise cv.Invalid("Automatic attenuation cannot be used when raw output is set")

    if config.get(CONF_ATTENUATION, None) == "auto" and config.get(CONF_SAMPLES, 1) > 1:
        raise cv.Invalid(
            "Automatic attenuation cannot be used when multisampling is set"
        )
    if config.get(CONF_ATTENUATION) == "11db":
        _LOGGER.warning(
            "`attenuation: 11db` is deprecated, use `attenuation: 12db` instead"
        )
        # Alter value here so `config` command prints the recommended change
        config[CONF_ATTENUATION] = _attenuation("12db")

    return config


def final_validate_config(config):
    if CORE.is_esp32:
        variant = get_esp32_variant()
        if (
            CONF_WIFI in fv.full_config.get()
            and config[CONF_PIN_A1][CONF_NUMBER]
            in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
            and config[CONF_PIN_A2][CONF_NUMBER]
            in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
            and config[CONF_PIN_A3][CONF_NUMBER]
            in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
            and config[CONF_PIN_V][CONF_NUMBER]
            in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
        ):
            raise cv.Invalid(
                f"{variant} doesn't support ADC on this pin when Wi-Fi is configured"
            )

    return config

CONF_ADC_CONTINOUS_ID = "adc_continous_id"

ADCSensor = adc_ns.class_(
    "ADCContinuousSensor", sensor.Sensor, cg.Component, voltage_sampler.VoltageSampler
)

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        ADCSensor,
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Required(CONF_PIN_A1): validate_adc_pin,
            cv.Required(CONF_PIN_A2): validate_adc_pin,
            cv.Required(CONF_PIN_A3): validate_adc_pin,
            cv.Required(CONF_PIN_V): validate_adc_pin,
            cv.Optional(CONF_RAW, default=False): cv.boolean,
            cv.SplitDefault(CONF_ATTENUATION, esp32="12db"): cv.All(
                cv.only_on_esp32, _attenuation
            ),
            cv.Optional(CONF_SAMPLES, default=1400): cv.int_range(min=1, max=4096),
            cv.Optional(CONF_SAMPLING_MODE, default="avg"): _sampling_mode,
            cv.Optional(CONF_FREQUENCY, default=20000): cv.int_range(min=20000, max=8000000),
            cv.Optional(CONF_UDP): cv.use_id(udp.UDPComponent),

        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    validate_config,
)

FINAL_VALIDATE_SCHEMA = final_validate_config


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    cg.add(var.set_output_raw(config[CONF_RAW]))
    cg.add(var.set_sample_count(config[CONF_SAMPLES]))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    if udp := config.get(CONF_UDP):
        udp_var = await cg.get_variable(udp)
        cg.add(var.set_udp(udp_var))

    if attenuation := config.get(CONF_ATTENUATION):
        if attenuation == "auto":
            cg.add(var.set_autorange(cg.global_ns.true))
        else:
            cg.add(var.set_attenuation(attenuation))

    if CORE.is_esp32:

        add_idf_sdkconfig_option("CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE", True)

        variant = get_esp32_variant()
        pin_num = config[CONF_PIN_V][CONF_NUMBER]
        if (
            variant in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel(0,chan))
        elif (
            variant in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel2(chan))                    
        pin_num = config[CONF_PIN_A1][CONF_NUMBER]
        if (
            variant in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel(1,chan))
        elif (
            variant in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel2(chan))

        pin_num = config[CONF_PIN_A2][CONF_NUMBER]
        if (
            variant in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel(2,chan))
        elif (
            variant in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel2(chan))

        pin_num = config[CONF_PIN_A3][CONF_NUMBER]
        if (
            variant in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel(3,chan))
        elif (
            variant in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL
            and pin_num in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
        ):
            chan = ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant][pin_num]
            cg.add(var.set_channel2(chan))
        cg.add_define("USE_OTA_STATE_CALLBACK")
