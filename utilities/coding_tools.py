import xml.etree.ElementTree as ET
from datetime import date
import urllib.request
import os

def generate_commands():
    """ Turns mavlink's common.xml MAV_CMD into python functions for commands
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

                        elif label == "NaN": # TODO: if NaN no value at all !!!
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
    """ Turns mavlink's common.xml MAV_CMD into python functions for messages
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

                        elif label == "NaN": # TODO: if NaN no value at all !!!
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

def generate_enums_init():
    """ Turns mavlink's common.xml enums into python functions to initiate
    """

    output_line = ''
    output_lines = 'from pymavlink import mavutil' + "\n\n"
    enum_name = ''
    
    url = 'https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml'
    response = urllib.request.urlopen(url)
    tree = ET.parse(response)

    root = tree.getroot()

    for enums in root.iter('enums'):
        for enum in enums.iter('enum'):
            if enum.get('name') != 'MAV_CMD':
                enum_name = enum.get('name')
                output_line = enum_name.lower() + ' = mavutil.mavlink.' + enum_name + '\n'
                possible_entries = ''
                for entry in enum.iter('entry'):
                    deprecated = entry.find('deprecated')
                    deprecation_comment = ''

                    if deprecated is not None:
                        for deprecta in entry.iter('deprecated'):
                            deprecation_comment = '# Deprecated since ' + deprecta.get('since') + \
                                ' and replaced by ' + deprecta.get('replaced_by') + '\n'

                    name = entry.get('name')
                    value = entry.get('value')
                    
                    if entry.find('description') is None:
                        description = ''
                    else:
                        description = entry.find('description').text

                        if description is None:
                            description = ''
                    
                    possible_entries += '# name: ' + name + '\n# value: ' + str(value) + '\n# description: ' + description + '\n\n'

                output_line = output_line + deprecation_comment + \
                    possible_entries + ' \n'
                    
            output_lines += output_line

    directory = os.path.dirname(__file__)
    path = r'{}/'.format(directory)

    today = date.today()
    d1 = today.strftime("%d_%m_%Y")

    with open(path + '/generated_enums_' + d1 + '.py', 'w') as file:
        file.write(output_lines)

def generate_minimal():
    """ Turns mavlink's common.xml enums into python functions to initiate
    """

    output_line = ''
    output_lines = 'import pandas as pd' + '\n' + 'from pymavlink import mavutil' + "\n\n"
    enum_name = ''
    
    url = 'https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/minimal.xml'
    response = urllib.request.urlopen(url)
    tree = ET.parse(response)

    root = tree.getroot()

    for enums in root.iter('enums'):
        for enum in enums.iter('enum'):
            # enum_category = enum.get('name')
            # enum_description = enum.get('description').text

            for entry in enum.iter('entry'):
                enum_name = entry.get('name')
                output_line = enum_name.lower().replace("mav_", "") + ' = mavutil.mavlink.' + enum_name + '\n'
                 
                possible_entries = ''
                deprecated = entry.find('deprecated')
                deprecation_comment = ''

                if deprecated is not None:
                    for deprecta in entry.iter('deprecated'):
                        deprecation_comment = '# Deprecated since ' + deprecta.get('since') + \
                            ' and replaced by ' + deprecta.get('replaced_by')  + '\n'

                    if deprecta.get('description') is not None:
                        deprecation_comment += '# Deprecation description ' + deprecta.get('description').text + '\n'

                value = entry.get('value')
                
                if entry.find('description') is None:
                    description = ''
                else:
                    description = enum_name.lower().replace("mav_", "") + '_description = "' + entry.find('description').text +'"'

                    if description is None:
                        description = ''
                       
                possible_entries += enum_name.lower().replace("mav_", "") + '_value = ' + str(value) + '\n' + description + '\n'

                output_line = output_line + deprecation_comment + \
                        possible_entries + '\n'
                    
                output_lines += output_line

    directory = os.path.dirname(__file__)
    path = r'{}/'.format(directory)

    today = date.today()
    d1 = today.strftime("%d_%m_%Y")

    with open(path + '/generated_minimal_' + d1 + '.py', 'w') as file:
        file.write(output_lines)
    pass

def update_mavlink_msg_cmd():
    # generate_messages()
    # generate_commands()
    # generate_enums_init()
    generate_minimal()

if __name__ == "__main__":
    update_mavlink_msg_cmd()