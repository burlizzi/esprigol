import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN, CONF_FREQUENCY
from esphome.core import CORE
from esphome.components.esp32 import add_idf_sdkconfig_option
from esphome import pins

DEPENDENCIES = []
AUTO_LOAD = []





rigol_ns = cg.esphome_ns.namespace("rigol")
Rigol = rigol_ns.class_("Rigol", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Rigol),
        cv.Required(CONF_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_FREQUENCY, default="20MHz"): cv.All(
            cv.frequency, cv.Range(min=611, max=2e6)
        ),

    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add_library("AsyncTCP-esphome", None)
    if CORE.using_esp_idf:
        add_idf_sdkconfig_option("CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED", False)
        add_idf_sdkconfig_option("CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED", False)

