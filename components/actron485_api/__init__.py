import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import web_server_base
from esphome.const import CONF_ID

CONF_CLIMATE_ID = "climate_id"
CONF_AUTH_TOKEN = "auth_token"
CONF_SENSOR_STALE_TIMEOUT = "sensor_stale_timeout"

AUTO_LOAD = ["web_server_base"]
DEPENDENCIES = ["web_server_base", "climate"]
CODEOWNERS = ["@wojt-janowski"]

actron485_api_ns = cg.esphome_ns.namespace("actron485_api")
Actron485Api = actron485_api_ns.class_("Actron485Api", cg.Component)

actron485_ns = cg.esphome_ns.namespace("actron485")
Actron485Climate = actron485_ns.class_("Actron485Climate")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Actron485Api),
        cv.Required(CONF_CLIMATE_ID): cv.use_id(Actron485Climate),
        cv.Optional(CONF_AUTH_TOKEN): cv.string_strict,
        # Auto-release wall-controller role for a zone if no
        # /temperature POST is received within this window. Set to 0 to
        # disable the safety.
        cv.Optional(
            CONF_SENSOR_STALE_TIMEOUT, default="10min"
        ): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    climate = await cg.get_variable(config[CONF_CLIMATE_ID])
    cg.add(var.set_climate(climate))

    if CONF_AUTH_TOKEN in config:
        cg.add(var.set_auth_token(config[CONF_AUTH_TOKEN]))

    cg.add(var.set_sensor_stale_timeout_ms(config[CONF_SENSOR_STALE_TIMEOUT]))

    # Pulled in by ESPHome's json component transitively, but declare
    # explicitly to make the dependency obvious and pin a major version.
    cg.add_library("bblanchon/ArduinoJson", "^7.0.0")
