import xml.etree.ElementTree as ET
from datetime import date
import urllib.request
import os

def generate_commands():
    """ Turns mavlink's common.xml MAV_CMD into python functions
        Errors in the output files may originate in common.xml eg: parameter n°x appears mutliple times
    """

    output_functions = 'from dronekit import Command\n' + \
                    'from pymavlink import mavutil\n\n'
    
    url = 'https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml'
    response = urllib.request.urlopen(url)
    tree = ET.parse(response)

    root = tree.getroot()

    for enums in root.iter('enums'):
        for enum in enums.iter('enum'):
            if enum.get('name') == 'MAV_CMD':
                for entry in enum.iter('entry'):
                    deprecated = entry.find('deprecated')
                    deprecation_comment = ''

                    if deprecated is not None:
                        for deprecta in entry.iter('deprecated'):
                            deprecation_comment = "Deprecated since " + deprecta.get('since') + \
                                ' and replaced by ' + deprecta.get('replaced_by') + '\n\t\t'

                    name = entry.get('name')
                    description = entry.find('description').text

                    if description is None:
                        description = ''

                    labels = ''
                    count_param = 0
                    arguments = ''

                    for parameter in entry.iter('param'):
                        label = parameter.get('label')
                        
                        if label is None:
                            if count_param == 6:
                                labels += '\t\t0)\n'
                            else:
                                labels += '\t\t0,\n'

                        elif label == "NaN":
                            if count_param == 6:
                                labels += '\t\tNaN)\n'
                            else:
                                labels += '\t\tNaN,\n'  

                        else: 
                            comment = ' # ' + parameter.text.strip()
                            reworded_label = label.lower().replace("/","_").replace(" ", "_")

                            if count_param == 6:
                                labels += '\t\t' + reworded_label + ')' + comment + '\n'
                            else:
                                labels += '\t\t' + reworded_label + ',' + comment + '\n'
                                
                            if  count_param > 0:
                                arguments += ', ' + reworded_label
                            else:
                                arguments += reworded_label
                            
                        count_param += 1

                    if count_param < 6:
                        delta = 6 - count_param

                        for n in range(0, delta):
                            if n < delta - 1:
                                labels += '\t\t0,\n'
                            else:
                                labels += '\t\t0)\n'

                    output_function = 'def ' + name.lower() + "(" + arguments + "):\n" + \
                        '\t""" ' + deprecation_comment + description + '\n\t"""\n\n' + \
                        '\tcmd = Command(\n' + \
                        '\t\t0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,\n' + \
                        '\t\tmavutil.mavlink.' + name + ",\n" + \
                        '\t\t0, 0,\n' + \
                        labels + "\n" + \
                        '\treturn cmd\n\n'

                    output_functions += output_function

    directory = os.path.dirname(__file__)
    path = r'{}/'.format(directory)

    today = date.today()
    d1 = today.strftime("%d_%m_%Y")

    with open(path + '/generated_commands_' + d1 + '.py', 'w') as file:
        file.write(output_functions)

def generate_messages():
    """ Turns mavlink's common.xml MAV_CMD into python functions
        Errors in the output files may originate in common.xml eg: parameter n°x appears mutliple times
    """

    output_functions = 'from pymavlink import mavutil\n\n'
    
    url = 'https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml'
    response = urllib.request.urlopen(url)
    tree = ET.parse(response)

    root = tree.getroot()

    for enums in root.iter('enums'):
        for enum in enums.iter('enum'):
            if enum.get('name') == 'MAV_CMD':
                for entry in enum.iter('entry'):
                    deprecated = entry.find('deprecated')
                    deprecation_comment = ''

                    if deprecated is not None:
                        for deprecta in entry.iter('deprecated'):
                            deprecation_comment = "Deprecated since " + deprecta.get('since') + \
                                ' and replaced by ' + deprecta.get('replaced_by') + '\n\t\t'

                    name = entry.get('name')
                    description = entry.find('description').text

                    if description is None:
                        description = ''

                    labels = ''
                    count_param = 0
                    arguments = 'self'

                    for parameter in entry.iter('param'):
                        label = parameter.get('label')
                        
                        if label is None:
                            if count_param == 6:
                                labels += '\t\t0)\n'
                            else:
                                labels += '\t\t0,\n'

                        elif label == "NaN":
                            if count_param == 6:
                                labels += '\t\tNaN)\n'
                            else:
                                labels += '\t\tNaN,\n'  

                        else: 
                            comment = ' # ' + parameter.text.strip()
                            reworded_label = label.lower().replace("/","_").replace(" ", "_")

                            if count_param == 6:
                                labels += '\t\t' + reworded_label + ')' + comment + '\n'
                            else:
                                labels += '\t\t' + reworded_label + ',' + comment + '\n'
                                
                            arguments += ', ' + reworded_label
                            
                        count_param += 1

                    if count_param < 6:
                        delta = 6 - count_param

                        for n in range(0, delta):
                            if n < delta - 1:
                                labels += '\t\t0,\n'
                            else:
                                labels += '\t\t0)\n'
        
                    output_function = 'def ' + name.lower() + "(" + arguments + "):\n" + \
                        '\t""" ' + deprecation_comment + description + '\n\t"""\n\n' + \
                        '\tmsg = self.vehicle.message_factory.command_long_encode(\n' + \
                        '\t\t0, 0,\n' + \
                        '\t\tmavutil.mavlink.' + name + ",\n" + \
                        '\t\t0,\n' + \
                        labels + "\n" + \
                        '\tself.vehicle.send_mavlink(msg)\n\n'

                    output_functions += output_function

    directory = os.path.dirname(__file__)
    path = r'{}/'.format(directory)

    today = date.today()
    d1 = today.strftime("%d_%m_%Y")

    with open(path + '/generated_messages_' + d1 + '.py', 'w') as file:
        file.write(output_functions)

def update_mavlink_msg_cmd():
    generate_messages()
    generate_commands()

if __name__ == "__main__":
    update_mavlink_msg_cmd()