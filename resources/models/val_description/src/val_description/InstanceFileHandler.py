import os
import xml.etree.ElementTree as xmlParser
import rospkg


class InstanceFileHandler():

    def __init__(self, instanceXmlFile):
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

        devicesRoot = self.instanceFileRoot.find('Devices')
        for device in devicesRoot.findall('Device'):
            self.devices.append(device)

        for mechanism in self.mechanisms:
            if mechanism.get('type') == 'simple':
                self.serialNumbers.append(
                    mechanism.find('SerialNumber').get('id'))
            elif mechanism.get('type') == 'complex':
                for actuator in mechanism.findall('Actuator'):
                    self.serialNumbers.append(
                        actuator.find('SerialNumber').get('id'))
            else:
                raise Exception('Invalid mechanism type in instance file!')

        for mechanism in self.mechanisms:
            if mechanism.get('type') == 'simple':
                self.nodes.append(mechanism.find('Node').get('id'))
            elif mechanism.get('type') == 'complex':
                for actuator in mechanism.findall('Actuator'):
                    self.nodes.append(actuator.find('Node').get('id'))
            else:
                raise Exception('Invalid mechanism type in instance file!')

    def buildConfigFileDictionary(self):
        self.configDictionary = {}
        for mechanism in self.mechanisms:
            nodeCoeffFileDictionary = {}
            if mechanism.get('type') == 'simple':
                tmpNode = mechanism.find('Node').get('id')
                tmpActuatorCoeffFile = mechanism.find(
                    'SerialNumber').get('id') + ".xml"

                nodeCoeffFileDictionary[tmpNode] = tmpActuatorCoeffFile

            elif mechanism.get('type') == 'complex':
                for actuator in mechanism.findall('Actuator'):
                    tmpNode = actuator.find('Node').get('id')
                    tmpActuatorCoeffFile = actuator.find(
                        'SerialNumber').get('id') + ".xml"

                    nodeCoeffFileDictionary[tmpNode] = tmpActuatorCoeffFile
            else:
                raise Exception('Invalid mechanism type')

            for node in nodeCoeffFileDictionary:
                actuatorCoeffFile = nodeCoeffFileDictionary[node]
                try:
                    actuatorFullFilePath = self.actuatorFileCoeffPath + \
                        "/" + actuatorCoeffFile
                    actuatorXmlCoeffFile = xmlParser.parse(actuatorFullFilePath)
                except IOError:
                    print 'Actuator coeff file %s does not exist' % (actuatorFullFilePath)
                    continue
                except xmlParser.ParseError:
                    raise Exception('Invalid XML in file %s' % (actuatorFullFilePath))

                self.configDictionary[node] = {}
                self.configDictionary[node]['configFiles'] = []

                try:
                    actuatorClassFile = actuatorXmlCoeffFile.find('ClassFile').get('id')
                except AttributeError:
                    raise Exception('ClassFile tag does not exist or is misspelled in actuator coeff file!')

                try:
                    actuatorControllerFile = actuatorXmlCoeffFile.find('ControllerFile').get('id')
                except AttributeError:
                    raise Exception('ControllerFile tag does not exist or is misspelled in actuator coeff file!')

                try:
                    actuatorLocationFile = actuatorXmlCoeffFile.find('LocationFile').get('id')
                except AttributeError:
                    raise Exception('LocationFile tag does not exist or is misspelled in actuator coeff file!')

                try:
                    actuatorSensorsFile = actuatorXmlCoeffFile.find('SensorsFile').get('id')
                except AttributeError:
                    raise Exception('SensorFile tag does not exist or is misspelled in actuator coeff file!')

                try:
                    actuatorSafetyFile = actuatorXmlCoeffFile.find(
                        'SafetyFile').get('id')
                except AttributeError:
                    raise Exception(
                        'SafetyFile tag does not exist or is misspelled in actuator coeff file!')

                try:
                    actuatorModeFile = actuatorXmlCoeffFile.find(
                        'ModeFile').get('id')
                except AttributeError:
                    raise Exception(
                        'ModeFile tag does not exist or is misspelled in actuator coeff file!')

                self.configDictionary[node][
                    'configFiles'].append(actuatorCoeffFile)
                self.configDictionary[node][
                    'configFiles'].append(actuatorClassFile)
                self.configDictionary[node][
                    'configFiles'].append(actuatorControllerFile)
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
                    print 'Class coeff file %s does not exist' % (classFullFilePath)
                    continue
                except xmlParser.ParseError:
                    raise Exception('Invalid XML in file %s' % (classFullFilePath))

                self.configDictionary[node][
                    'firmware'] = classXmlCoeffFile.find('Processor').get('id')
                self.configDictionary[node][
                    'type'] = classXmlCoeffFile.find('Type').get('id')
                self.configDictionary[node]['location'] = node

    def getInstanceRoot(self):
        return self.instanceFileRoot

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

    def getActuatorCoeffFiles(self):
        serialNumbers = self.getSerialNumbers()
        coeffFiles = []
        for serialNumber in serialNumbers:
            coeffFiles.append(serialNumber + '.xml')

        return coeffFiles

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
                    print('Could not open xml file: {}'.format(e))
                except xmlParser.ParseError as e:
                    print('Could not parse xml file: {}'.format(e))
            if not cfgs:
                raise Exception('No config values found!')
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
            print "\n Target {} doesn't exist, skipping! \n ".format(target)
            dictionary = dict()
            return dictionary

    def loadXMLCoeffs(self, fname):
        ''' Parse XML file and return name:value dict of coeffs'''
        result = ""
        for root, dirs, files, in os.walk(self.coeffFileRootPath):
            if fname in files:
                result = os.path.join(root, fname)

        coeffs = {}
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
