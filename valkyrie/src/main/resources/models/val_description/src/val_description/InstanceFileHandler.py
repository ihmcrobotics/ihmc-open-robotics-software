import os
import xml.etree.ElementTree as xmlParser
import rospkg
import logging


class InstanceFileHandler():

    def __init__(self, instanceXmlFile):

        self.logger = logging.getLogger(__name__)

        self.instanceFile = xmlParser.parse(instanceXmlFile)
        self.instanceFileRoot = self.instanceFile.getroot()
        self.robotChildren = []
        for child in self.instanceFileRoot:
            self.robotChildren.append(child)

        # Setup paths to the various coeff files
        rospack = rospkg.RosPack()
        self.valDescriptionPackagePath = rospack.get_path('val_description')
        self.coeffFileRootPath = self.valDescriptionPackagePath + '/instance/coefficients'

        self.actuatorFileCoeffPath = self.coeffFileRootPath + '/actuators'
        self.calibrationFileCoeffPath = self.coeffFileRootPath + '/calibration'
        self.classFileCoeffPath = self.coeffFileRootPath + '/class'
        self.controllerFileCoeffPath = self.coeffFileRootPath + '/controllers'
        self.locationFileCoeffPath = self.coeffFileRootPath + '/location'
        self.modeFileCoeffPath = self.coeffFileRootPath + '/modes'
        self.safetyFileCoeffPath = self.coeffFileRootPath + '/safety'
        self.sensorFileCoeffPath = self.coeffFileRootPath + '/sensors'

        self.processInstanceFile()

        self.buildConfigFileDictionary()

    def processInstanceFile(self):
        self.mechanisms = []
        self.channels = []
        self.devices = []
        self.nodes = []
        self.serialNumbers = []

        mechanismsRoot = self.instanceFileRoot.find('Mechanisms')
        for mechanism in mechanismsRoot.findall('Mechanism'):
            self.mechanisms.append(mechanism)

        channelsRoot = self.instanceFileRoot.find('Channels')
        for channel in channelsRoot.findall('Channel'):
            self.channels.append(channel)

        try:
            devicesRoot = self.instanceFileRoot.find('Devices')
        except AttributeError as e:
            msg = "Instance file doesn't contain the Devices tag or it is misspelled!"
            print msg
            self.logger.error(msg)

        for device in devicesRoot.findall('Device'):
            self.devices.append(device)

        for mechanism in self.mechanisms:
            if mechanism.get('type') == 'simple' or mechanism.get('type') == 'forearm':
                self.serialNumbers.append(
                    mechanism.find('SerialNumber').get('id'))
            elif mechanism.get('type') == 'complex':
                for actuator in mechanism.findall('Actuator'):
                    self.serialNumbers.append(
                        actuator.find('SerialNumber').get('id'))
            else:
                msg = 'Invalid mechanism type in instance file!'
                self.logger.error(msg)
                raise Exception(msg)

        for mechanism in self.mechanisms:
            if mechanism.get('type') == 'simple':
                self.nodes.append(mechanism.find('Node').get('id'))
            elif mechanism.get('type') == 'complex':
                for actuator in mechanism.findall('Actuator'):
                    self.nodes.append(actuator.find('Node').get('id'))
            elif mechanism.get('type') == 'forearm':
                athenaNodesRoot = mechanism.find('Nodes')
                self.nodes.append(athenaNodesRoot.find('Athena1').get('id'))
                self.nodes.append(athenaNodesRoot.find('Athena2').get('id'))

            else:
                msg = 'Invalid mechanism type in instance file!'
                self.logger.error(msg)
                raise Exception(msg)

    def buildConfigFileDictionary(self):
        self.configDictionary = {}
        self.nodeCoeffFileDictionary = {}
        self.actuatorNameCoeffFileDictionary = {}
        self.forearmCoeffDictionary = {}
        for mechanism in self.mechanisms:
            if mechanism.get('type') == 'simple':
                tmpNode = mechanism.find('Node').get('id')
                tmpActuatorCoeffFile = mechanism.find(
                    'SerialNumber').get('id') + ".xml"

                self.nodeCoeffFileDictionary[tmpNode] = tmpActuatorCoeffFile
                self.actuatorNameCoeffFileDictionary[mechanism.get('id')] = tmpActuatorCoeffFile

            elif mechanism.get('type') == 'complex':
                for actuator in mechanism.findall('Actuator'):
                    tmpNode = actuator.find('Node').get('id')
                    tmpActuatorCoeffFile = actuator.find(
                        'SerialNumber').get('id') + ".xml"

                    self.nodeCoeffFileDictionary[tmpNode] = tmpActuatorCoeffFile
                    self.actuatorNameCoeffFileDictionary[actuator.get('id')] = tmpActuatorCoeffFile

            elif mechanism.get('type') == 'forearm':
                athenaSerialNumber = mechanism.find(
                    'SerialNumber').get('id')

                athena1CoeffFile = athenaSerialNumber + "_athena1.xml"
                athena2CoeffFile = athenaSerialNumber + "_athena2.xml"

                athena1CoeffDictionary = {}
                athena2CoeffDictionary = {}

                athena1CoeffDictionary = self.loadXMLCoeffs(athena1CoeffFile)
                athena2CoeffDictionary = self.loadXMLCoeffs(athena2CoeffFile)

                if not athena1CoeffDictionary or not athena2CoeffDictionary:
                    self.logger.error('Skipping athenas because coeff dictionaries are empty, check your instance file and coeff files!')
                    continue

                self.forearmCoeffDictionary[mechanism.find('Nodes').find('Athena1').get('id')] = {}
                for coeff, coeffDictionary in athena1CoeffDictionary.iteritems():
                    self.forearmCoeffDictionary[mechanism.find('Nodes').find('Athena1').get('id')][coeff] = coeffDictionary['value']

                self.forearmCoeffDictionary[mechanism.find('Nodes').find('Athena2').get('id')] = {}
                for coeff, coeffDictionary in athena2CoeffDictionary.iteritems():
                    self.forearmCoeffDictionary[mechanism.find('Nodes').find('Athena2').get('id')][coeff] = coeffDictionary['value']

            else:
                msg = 'Invalid mechanism type'
                self.logger.error(msg)
                raise Exception(msg)

        for node in self.nodeCoeffFileDictionary:
            actuatorCoeffFile = self.nodeCoeffFileDictionary[node]
            try:
                actuatorFullFilePath = self.actuatorFileCoeffPath + \
                    "/" + actuatorCoeffFile
                actuatorXmlCoeffFile = xmlParser.parse(actuatorFullFilePath)
            except IOError:
                msg = 'Actuator coeff file %s does not exist' % (actuatorFullFilePath)
                self.logger.warn(msg)
                continue
            except xmlParser.ParseError:
                msg = 'Invalid XML in file %s' % (actuatorFullFilePath)
                self.logger.error(msg)
                raise Exception(msg)

            self.configDictionary[node] = {}
            self.configDictionary[node]['configFiles'] = []

            try:
                actuatorClassFile = actuatorXmlCoeffFile.find('ClassFile').get('id')
                try:
                    actuatorSubClassFile = actuatorXmlCoeffFile.find('ClassFile').find('SubClassFile').get('id')
                    msg = 'Found SubClassFile ' + actuatorSubClassFile + '!'
                    self.logger.info(msg)
                except AttributeError:
                    actuatorSubClassFile = None
                    # Coeffs are not required to specify a subclass
                    
            except AttributeError:
                msg = 'ClassFile tag does not exist or is misspelled in actuator coeff file!'
                self.logger.error(msg)
                raise Exception(msg)

            try:
                actuatorControllerFile = actuatorXmlCoeffFile.find('ControllerFile').get('id')
                try:
                    actuatorSubControllerFile = actuatorXmlCoeffFile.find('ControllerFile').find('SubControllerFile').get('id')
                    msg = 'Found SubControllerFile ' + actuatorSubControllerFile + '!'
                    self.logger.info(msg)
                except AttributeError:
                    # Coeffs are not required to specify a subclass
                    actuatorSubControllerFile = None
            except AttributeError:
                msg = 'ControllerFile tag does not exist or is misspelled in actuator coeff file!'
                self.logger.error(msg)
                raise Exception(msg)

            try:
                actuatorLocationFile = actuatorXmlCoeffFile.find('LocationFile').get('id')
            except AttributeError:
                msg = 'LocationFile tag does not exist or is misspelled in actuator coeff file!'
                self.logger.error(msg)
                raise Exception(msg)

            try:
                actuatorSensorsFile = actuatorXmlCoeffFile.find('SensorsFile').get('id')
            except AttributeError:
                msg = 'SensorFile tag does not exist or is misspelled in actuator coeff file!'
                self.logger.error(msg)
                raise Exception(msg)

            try:
                actuatorSafetyFile = actuatorXmlCoeffFile.find(
                    'SafetyFile').get('id')
            except AttributeError:
                msg = 'SafetyFile tag does not exist or is misspelled in actuator coeff file!'
                self.logger.error(msg)
                raise Exception(msg)

            try:
                actuatorModeFile = actuatorXmlCoeffFile.find(
                    'ModeFile').get('id')
            except AttributeError:
                msg = 'ModeFile tag does not exist or is misspelled in actuator coeff file!'
                self.logger.error(msg)
                raise Exception(msg)

            self.configDictionary[node][
                'configFiles'].append(actuatorCoeffFile)
            self.configDictionary[node][
                'configFiles'].append(actuatorClassFile)
            if actuatorSubClassFile:
                self.configDictionary[node][
                    'configFiles'].append(actuatorSubClassFile)
            self.configDictionary[node][
                'configFiles'].append(actuatorControllerFile)
            if actuatorSubControllerFile:
                self.configDictionary[node][
                    'configFiles'].append(actuatorSubControllerFile)
            self.configDictionary[node][
                'configFiles'].append(actuatorLocationFile)
            self.configDictionary[node][
                'configFiles'].append(actuatorSensorsFile)
            self.configDictionary[node][
                'configFiles'].append(actuatorSafetyFile)
            self.configDictionary[node]['configFiles'].append(actuatorModeFile)

            try:
                classFullFilePath = self.classFileCoeffPath + \
                    "/" + actuatorClassFile
                classXmlCoeffFile = xmlParser.parse(classFullFilePath)
            except IOError:
                self.logger.warn('Class coeff file %s does not exist' % (classFullFilePath))
                continue
            except xmlParser.ParseError:
                msg = 'Invalid XML in file %s' % (classFullFilePath)
                self.logger.error(msg)
                raise Exception(msg)

            self.configDictionary[node][
                'firmware'] = classXmlCoeffFile.find('Processor').get('id')
            self.configDictionary[node][
                'type'] = classXmlCoeffFile.find('Type').get('id')
            self.configDictionary[node]['location'] = node

    def getInstanceRoot(self):
        return self.instanceFileRoot

    def getForearmCoeffDictionary(self):
        return self.forearmCoeffDictionary

    def getMechanisms(self):
        return self.mechanisms

    def getChannels(self):
        return self.channels

    def getNodeNames(self):
        return self.nodes

    def getDevices(self):
        return self.devices

    def getSerialNumbers(self):
        return self.serialNumbers

    def getSerialNumberByActuatorName(self, actuatorName):
        try:
            coeffFile = self.actuatorNameCoeffFileDictionary[actuatorName]
            return coeffFile
        except KeyError as e:
            msg = 'Actuator ' + actuatorName + ' not found in instance file!'
            self.logger.error(msg)
            raise Exception(msg)

    def getActuatorCoeffFiles(self):
        serialNumbers = self.getSerialNumbers()
        coeffFiles = []
        for serialNumber in serialNumbers:
            coeffFiles.append(serialNumber + '.xml')

        return coeffFiles

    def getNodeActuatorCoeffFileDictionary(self):
        return self.nodeCoeffFileDictionary

    def getActuatorCoeffFileByNode(self, nodeName):
        try:
            actuatorCoeffFile = self.nodeCoeffFileDictionary[nodeName]
        except:
            msg = 'Node name ' + nodeName + 'not found in instance file!'
            self.logger.error(msg)
            raise Exception(msg)

        return actuatorCoeffFile

    def getInstanceConfig(self):
        return self.configDictionary

    def getConfig(self, target):
        return self.gatherCoeffs(target)

    def getType(self, target):
        return self.configDictionary[target]['type']

    def getFirmware(self, nodeName):
        return self.configDictionary[nodeName]['firmware']

    def getNodeType(self, nodeName):
        return self.configDictionary[nodeName]['type']

    def getActuatorSerialNumberByNode(self, nodeName):
        for mechanism in self.mechanisms:
            if mechanism.get('type') == 'simple':
                mechanismNode = mechanism.find('Node').get('id')
                if mechanismNode == nodeName:
                    return mechanism.find('SerialNumber').get('id')
            elif mechanism.get('type') == 'complex':
                for actuator in mechanism.findall('Actuator'):
                    actuatorNode = actuator.find('Node').get('id')
                    if actuatorNode == nodeName:
                        return actuator.find('SerialNumber').get('id')
            else:
                raise Exception('Node name not found in instance file!')

    def gatherCoeffs(self, target):
        ''' Given a list of coeff files resolve them to a single dictionary '''
        cfgs = []

        if target in self.configDictionary.keys():
            for f in self.configDictionary[target]['configFiles']:
                try:
                    filetype = os.path.splitext(f)[1].lower()
                    if filetype == '.xml':
                        cfgs.append(self.loadXMLCoeffs(f))
                    elif filetype == '.json' or filetype == '.yaml':
                        with open(f, 'r') as fs:
                            cfgs.append(yaml.load(fs))
                    else:
                        print('Unsupported coeff format {}'.format(filetype))
                except IOError as e:
                    self.logger.warn('Could not open xml file: {}'.format(e))
                except xmlParser.ParseError as e:
                    msg = 'Could not parse xml file {}: {}'.format(f, e)
                    self.logger.warn(msg)
            if not cfgs:
                msg = 'No config values found!'
                self.logger.error(msg)
                raise Exception(msg)
            cfgs.reverse()  # makes sure precedence works in next operation
            retCfg = reduce(lambda x, y: dict(x.items() + y.items()), cfgs)
            # return dictionary of key,values: {'Coeff_x': value}
            coeffs = {}
            disabled = []
            for k, v in retCfg.iteritems():
                if type(v) == dict:
                    if not v.get('disabled', False):
                        coeffs[k] = v['value']
                    else:
                        disabled.append(k)
                else:
                    coeffs[k] = v
            if disabled:
                print('Bypassing disabled coeffs: ' +
                      ', '.join([coeff for coeff in disabled]))

            return coeffs
        else:
            self.logger.warn("Target {} doesn't exist, skipping!".format(target))
            dictionary = dict()
            return dictionary

    def loadXMLCoeffs(self, fname):
        ''' Parse XML file and return name:value dict of coeffs'''
        result = ""
        for root, dirs, files, in os.walk(self.coeffFileRootPath):
            if fname in files:
                result = os.path.join(root, fname)

        coeffs = {}

        if result == "":
            self.logger.error('Coeff file name {} was not found, skipping! Check that the file exists!'.format(fname))
            return coeffs

        xmlCoeffObject = xmlParser.parse(result)
        for coeff in xmlCoeffObject.iter('Coeff'):
            coeffName = coeff.get('id')
            coeffs[coeffName] = {}
            coeffs[coeffName]['source'] = fname
            coeffs[coeffName]['value'] = coeffValue = float(coeff.get('value'))
            if coeff.get('group') is not None:
                coeffs[coeffName]['group'] = coeff.get('group')
            if coeff.get('disabled') is not None:
                if coeff.get('disabled').lower() == 'true':
                    coeffs[coeffName]['disabled'] = True
                else:
                    coeffs[coeffName]['disabled'] = False

        return coeffs
