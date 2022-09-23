const {
    fromZigbeeConverters,
    toZigbeeConverters,
    exposes
} = require('zigbee-herdsman-converters');

const bind = async (endpoint, target, clusters) => {
    for (const cluster of clusters) {
        await endpoint.bind(cluster, target);
    }
};

const ACCESS_STATE = 0b001, ACCESS_WRITE = 0b010, ACCESS_READ = 0b100;

const withEpPreffix = (converter) => ({
    ...converter,
    convert: (model, msg, publish, options, meta) => {
        const epID = msg.endpoint.ID;
        const converterResults = converter.convert(model, msg, publish, options, meta) || {};
        return Object.keys(converterResults)
            .reduce((result, key) => {
                result[`${key}_${epID}`] = converterResults[key];
                return result;
            }, {});
    },
});

const postfixWithEndpointName = (name, msg, definition) => {
    if (definition.meta && definition.meta.multiEndpoint) {
        const endpointName = definition.hasOwnProperty('endpoint') ?
            getKey(definition.endpoint(msg.device), msg.endpoint.ID) : msg.endpoint.ID;
        return `${name}_${endpointName}`;
    } else {
        return name;
    }
};

const fz = {
    illuminance: {
        cluster: 'msIlluminanceMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.hasOwnProperty('measuredValue')) {
                const illuminance = msg.data['measuredValue'];
                const property = postfixWithEndpointName('illuminance', msg, model);
                return {[property]: msg.data.measuredValue};
            }
        },
    },
    battery_config: {
        cluster: 'genPowerCfg',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty(0xF003)) {
                result.battery_period = msg.data[0xF003];
            }
            return result;
        },
    },
    illuminance_config: {
        cluster: 'msIlluminanceMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty(0xF000)) {
                result.illuminance_sensitivity = msg.data[0xF000];
            }
            if (msg.data.hasOwnProperty(0xF001)) {
                result.illuminance_change = msg.data[0xF001];
            }
            if (msg.data.hasOwnProperty(0xF002)) {
                result.illuminance_period = msg.data[0xF002];
            }
            return result;
        },
    },
    temperature_config: {
        cluster: 'msTemperatureMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty('measuredValue')) {
                result.temperature = msg.data.measuredValue/100;
            }
            if (msg.data.hasOwnProperty(0xF001)) {
                result.temperature_change = msg.data[0xF001]/100;
            }
            if (msg.data.hasOwnProperty(0xF002)) {
                result.temperature_period = msg.data[0xF002];
            }
            return result;
        },
    },
    pressure_config: {
        cluster: 'msPressureMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty('scaledValue')) {
                result.pressure = msg.data.scaledValue/10;
            }
            if (msg.data.hasOwnProperty(0xF001)) {
                result.pressure_change = msg.data[0xF001]/10;
            }
            if (msg.data.hasOwnProperty(0xF002)) {
                result.pressure_period = msg.data[0xF002];
            }
            return result;
        },
    },
    humidity_config: {
        cluster: 'msRelativeHumidity',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty('measuredValue')) {
                result.humidity = msg.data.measuredValue/100;
            }
            if (msg.data.hasOwnProperty(0xF001)) {
                result.humidity_change = msg.data[0xF001]/100;
            }
            if (msg.data.hasOwnProperty(0xF002)) {
                result.humidity_period = msg.data[0xF002];
            }
            return result;
        },
    },
    occupancy: {
        cluster: 'msOccupancySensing',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.hasOwnProperty('occupancy')) {
                return {occupancy: msg.data.occupancy === 1};
            }
        },
    },
    occupancy_config: {
        cluster: 'msOccupancySensing',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty('pirOToUDelay')) {
              result.occupancy_timeout = msg.data.pirOToUDelay;
            }
            if (msg.data.hasOwnProperty('pirUToODelay')) {
              result.unoccupancy_timeout = msg.data.pirUToODelay;
            }
            return result;
        },
    },
    contact: {
        cluster: 'genBinaryInput',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            return {contact: msg.data['presentValue'] !== 0};
        },
    },
};

const tz = {
    occupancy_timeout: {
        // set delay after motion detector changes from occupied to unoccupied
        key: ['occupancy_timeout'],
        convertSet: async (entity, key, value, meta) => {
            value *= 1;
            const thirdEndpoint = meta.device.getEndpoint(3);
            await thirdEndpoint.write('msOccupancySensing', {pirOToUDelay: value});
            return {state: {occupancy_timeout: value}};
        },
        convertGet: async (entity, key, meta) => {
            const thirdEndpoint = meta.device.getEndpoint(3);
            await thirdEndpoint.read('msOccupancySensing', ['pirOToUDelay']);
        },
    },
    unoccupancy_timeout: {
        // set delay after motion detector changes from unoccupied to occupied
        key: ['unoccupancy_timeout'],
        convertSet: async (entity, key, value, meta) => {
            value *= 1;
            const thirdEndpoint = meta.device.getEndpoint(3);
            await thirdEndpoint.write('msOccupancySensing', {pirUToODelay: value});
            return {state: {unoccupancy_timeout: value}};
        },
        convertGet: async (entity, key, meta) => {
            const thirdEndpoint = meta.device.getEndpoint(3);
            await thirdEndpoint.read('msOccupancySensing', ['pirUToODelay']);
        },
    },
    illuminance_config: {
        // set illuminance sensitivity
        key: ['illuminance_sensitivity', 'illuminance_change', 'illuminance_period'],
        convertSet: async (entity, key, value, meta) => {
            value *= 1;
            const fourthEndpoint = meta.device.getEndpoint(4);
            const payloads = {
                illuminance_sensitivity: ['msIlluminanceMeasurement', {0xF000: {value, type: 0x21}}],
                illuminance_change: ['msIlluminanceMeasurement', {0xF001: {value, type: 0x21}}],
                illuminance_period: ['msIlluminanceMeasurement', {0xF002: {value, type: 0x21}}],
            };
            await fourthEndpoint.write(payloads[key][0], payloads[key][1]);
            return {
                state: {[key]: value},
            };
        },
        convertGet: async (entity, key, meta) => {
            const fourthEndpoint = meta.device.getEndpoint(4);
            const payloads = {
                illuminance_sensitivity: ['msIlluminanceMeasurement', 0xF000],
                illuminance_change: ['msIlluminanceMeasurement', 0xF001],
                illuminance_period: ['msIlluminanceMeasurement', 0xF002],
            };
            await fourthEndpoint.read(payloads[key][0], [payloads[key][1]]);
        },
    },
    change_period: {
        // set minAbsoluteChange
        key: ['temperature_change', 'pressure_change', 'humidity_change', 'temperature_period', 'pressure_period', 'humidity_period', 'battery_period'],
        convertSet: async (entity, key, value, meta) => {
            value *= 1;
            const temp_value = value;
            if ([key] == 'pressure_change'){
              value *= 10;
            }
            if ([key] == 'temperature_change'){
              value *= 100;
            }
            if ([key] == 'humidity_change'){
              value *= 100;
            }
            const payloads = {
                temperature_change: ['msTemperatureMeasurement', {0xF001: {value, type: 0x21}}],
                pressure_change: ['msPressureMeasurement', {0xF001: {value, type: 0x21}}],
                humidity_change: ['msRelativeHumidity', {0xF001: {value, type: 0x21}}],
                temperature_period: ['msTemperatureMeasurement', {0xF002: {value, type: 0x21}}],
                pressure_period: ['msPressureMeasurement', {0xF002: {value, type: 0x21}}],
                humidity_period: ['msRelativeHumidity', {0xF002: {value, type: 0x21}}],
                battery_period: ['genPowerCfg', {0xF003: {value, type: 0x21}}],
            };
            await entity.write(payloads[key][0], payloads[key][1]);
            return {
                state: {[key]: temp_value},
            };
        },
        convertGet: async (entity, key, meta) => {
            const payloads = {
                temperature_change: ['msTemperatureMeasurement', 0xF001],
                pressure_change: ['msPressureMeasurement', 0xF001],
                humidity_change: ['msRelativeHumidity', 0xF001],
                temperature_period: ['msTemperatureMeasurement', 0xF002],
                pressure_period: ['msPressureMeasurement', 0xF002],
                humidity_period: ['msRelativeHumidity', 0xF002],
                battery_period: ['genPowerCfg', 0xF003],
            };
            await entity.read(payloads[key][0], [payloads[key][1]]);
        },
    },
};

const device = {
        zigbeeModel: ['DIYRuZ_Motion'],
        model: 'DIYRuZ_Motion',
        vendor: 'DIYRuZ',
        description: '[Motion sensor](https://github.com/koptserg/motion)',
        supports: 'temperature, humidity, illuminance, contact, pressure, battery, occupancy',
        fromZigbee: [
            fz.battery_config,
            fz.temperature_config,
            fz.humidity_config,
            fz.illuminance,
            fz.illuminance_config,
            fz.pressure_config,
            fromZigbeeConverters.battery,
            fz.contact,
            fz.occupancy,
            fz.occupancy_config,
        ],
        toZigbee: [
            tz.occupancy_timeout,
            tz.unoccupancy_timeout,
            tz.illuminance_config,
            tz.change_period,
            toZigbeeConverters.factory_reset,
        ],
        meta: {
            configureKey: 1,
            multiEndpoint: true,
        },
        configure: async (device, coordinatorEndpoint) => {
            const firstEndpoint = device.getEndpoint(1);
            const secondEndpoint = device.getEndpoint(2);
            const thirdEndpoint = device.getEndpoint(3);
            const fourthEndpoint = device.getEndpoint(4);
            await bind(firstEndpoint, coordinatorEndpoint, [
                'genPowerCfg',
                'msTemperatureMeasurement',
                'msRelativeHumidity',
                'msPressureMeasurement',
                'msIlluminanceMeasurement',
            ]);
            await bind(secondEndpoint, coordinatorEndpoint, [
//                'genOnOff',
                'genBinaryInput',
            ]);
            await bind(thirdEndpoint, coordinatorEndpoint, [
//                'genOnOff',
                'msOccupancySensing',
            ]);
            await bind(fourthEndpoint, coordinatorEndpoint, [
                'msIlluminanceMeasurement',
            ]);

        const genPowerCfgPayload = [{
                attribute: 'batteryVoltage',
                minimumReportInterval: 0,
                maximumReportInterval: 3600,
                reportableChange: 0,
            },
            {
                attribute: 'batteryPercentageRemaining',
                minimumReportInterval: 0,
                maximumReportInterval: 3600,
                reportableChange: 0,
            }
        ];

        const msBindPayload = [{
            attribute: 'measuredValue',
            minimumReportInterval: 0,
            maximumReportInterval: 3600,
            reportableChange: 0,
        }];
        const msTemperatureBindPayload = [{
            attribute: 'measuredValue',
            minimumReportInterval: 0,
            maximumReportInterval: 3600,
            reportableChange: 0,
        }];
        const genBinaryInputBindPayload = [{
            attribute: 'presentValue',
            minimumReportInterval: 0,
            maximumReportInterval: 3600,
            reportableChange: 0,
        }];
        const msOccupancySensingBindPayload = [{
            attribute: 'occupancy',
            minimumReportInterval: 0,
            maximumReportInterval: 3600,
            reportableChange: 0,
        }];
            await firstEndpoint.configureReporting('genPowerCfg', genPowerCfgPayload);
            await firstEndpoint.configureReporting('msTemperatureMeasurement', msTemperatureBindPayload);
            await firstEndpoint.configureReporting('msRelativeHumidity', msBindPayload);
            await firstEndpoint.configureReporting('msPressureMeasurement', msBindPayload);
            await firstEndpoint.configureReporting('msIlluminanceMeasurement', msBindPayload);
            await secondEndpoint.configureReporting('genBinaryInput', genBinaryInputBindPayload);
            await thirdEndpoint.configureReporting('msOccupancySensing', msOccupancySensingBindPayload);
            await fourthEndpoint.configureReporting('msIlluminanceMeasurement', msBindPayload);
        },
        exposes: [
            exposes.numeric('battery', ACCESS_STATE).withUnit('%').withDescription('Remaining battery in %').withValueMin(0).withValueMax(100),
            exposes.numeric('temperature', ACCESS_STATE).withUnit('°C').withDescription('Measured temperature value'), 
            exposes.numeric('humidity', ACCESS_STATE).withUnit('%').withDescription('Measured relative humidity'),
            exposes.numeric('pressure', ACCESS_STATE).withUnit('hPa').withDescription('The measured atmospheric pressure'),
            exposes.numeric('illuminance_1', ACCESS_STATE).withDescription('Raw measured illuminance LDR'), 
            exposes.numeric('illuminance_4', ACCESS_STATE).withUnit('lx').withDescription('Measured illuminance in lux BH1750'),
            exposes.binary('contact', ACCESS_STATE).withDescription('Indicates if the contact is closed (= true) or open (= false)'), 
            exposes.binary('occupancy', ACCESS_STATE).withDescription('Indicates whether the device detected occupancy'), 
            exposes.numeric('occupancy_timeout', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('sec').withDescription('Delay occupied to unoccupied + 10 sec adaptation'),
            exposes.numeric('unoccupancy_timeout', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('sec').withDescription('Delay unoccupied to occupied'),
            exposes.numeric('illuminance_sensitivity', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withDescription('Illuminance level sensitivity 31 - 254 (default = 69)'),
            exposes.numeric('temperature_change', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('°C').withDescription('Temperature min absolute change (default = 0,5 °C)'),
            exposes.numeric('temperature_period', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('sec').withDescription('Temperature measurement period (default = 10 sec)'),
            exposes.numeric('pressure_change', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('hPa').withDescription('Pressure min absolute change (default = 0,1 hPa)'),
            exposes.numeric('pressure_period', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('sec').withDescription('Pressure measurement period (default = 10 sec)'),
            exposes.numeric('humidity_change', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('%').withDescription('Humidity min absolute change (default = 10%)'),
            exposes.numeric('humidity_period', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('sec').withDescription('Humidity measurement period (default = 10 sec)'),
            exposes.numeric('illuminance_change', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('lx').withDescription('Illuminance BH1750 min absolute change (default = 10 lux)'),
            exposes.numeric('illuminance_period', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('sec').withDescription('Illuminance BH1750 measurement period (default = 10 sec)'),
            exposes.numeric('battery_period', ACCESS_STATE | ACCESS_WRITE | ACCESS_READ).withUnit('min').withDescription('Battery report period (default = 30 min)'),
        ],
};

module.exports = device;