import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN, CONF_FREQUENCY, CONF_PORT
from esphome.core import CORE
from esphome.components.esp32 import add_idf_sdkconfig_option
from esphome import pins
from esphome.components import tcp_server
from esphome.components.tcp_server import CONF_TCP_SERVER_ID
from esphome.components import adc_continous
from esphome.components.adc_continous.sensor import CONF_ADC_CONTINOUS_ID
DEPENDENCIES = []
AUTO_LOAD = ["adc_continous","tcp_server"]





rigol_ns = cg.esphome_ns.namespace("rigol")
Rigol = rigol_ns.class_("Rigol", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Rigol),
        cv.GenerateID(CONF_TCP_SERVER_ID): cv.use_id(
                tcp_server.TcpServer
            ),
        cv.GenerateID(CONF_ADC_CONTINOUS_ID): cv.use_id(
                adc_continous.sensor.ADCSensor
            ),

        #cv.Optional(CONF_PIN): pins.internal_gpio_input_pin_number,
        #cv.Optional(CONF_FREQUENCY, default="20MHz"): cv.All(
        #    cv.frequency, cv.Range(min=611, max=2e6)
        #),

    }
)

async def to_code(config):
    tcp = await cg.get_variable(config[CONF_TCP_SERVER_ID])
    adc = await cg.get_variable(config[CONF_ADC_CONTINOUS_ID])
    var = cg.new_Pvariable(config[CONF_ID],tcp,adc)
    await cg.register_component(var, config)
    #cg.add_library("AsyncTCP-esphome", None)
    #if CORE.using_esp_idf:
    #    add_idf_sdkconfig_option("CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED", False)
    #    add_idf_sdkconfig_option("CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED", False)

