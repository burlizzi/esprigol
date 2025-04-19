import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID,CONF_PORT

DEPENDENCIES = []

CONF_TCP_SERVER_ID = "tcp_server_id"
tcp_ns = cg.esphome_ns.namespace("tcp_server")
TcpServer = tcp_ns.class_("TcpServer", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TcpServer),
        cv.Optional(CONF_PORT,default=5555): cv.Any(0, cv.port),
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID],config[CONF_PORT])
    await cg.register_component(var, config)
