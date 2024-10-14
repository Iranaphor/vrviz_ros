#! /usr/bin/env python3

import importlib

def get_rosmsg_obj(ros_pkg, class_name):
    module = importlib.import_module(ros_pkg+".msg")
    rosmsg_obj = getattr(module, class_name)
    return rosmsg_obj


def convert_ros_message_to_dictionary(rosmsg):
    """ Recursively break apart the rosmsg into a layered dictionary """
    dict_obj = dict()

    # Loop through each property in the rosmsg
    #try:
    #    rosmsg._fields_and_field_types.items()
    #except:
    #   return

    for property_name, property_type in rosmsg._fields_and_field_types.items():
        property = getattr(rosmsg, property_name)

        # If having problems, debug using this:
        #if len(str(property)) > 1000:
        #    print(property_name, property_type, str(property)[:1000])
        #else:
        #   print(property_name, property_type, str(property))

        # If the property is a list of objects i.e. sequence<uint8>
        if 'sequence' in property_type:
            data = []
            subtype = property_type.split('<')[1].split('>')[0]
            if '/' in subtype or '[' in subtype:
                for item in property:
                    data += [convert_ros_message_to_dictionary(item)]
            else:
                for item in property:
                    data += [item]

        # If the property is another rosmsg object i.e. geometry_msg/msg/Point
        elif '/' in property_type:
            data = convert_ros_message_to_dictionary(property)

        # If the property is a list i.e. double[32]
        elif '[' in property_type:
            data = []
            for item in property:
                data += [item]

        # Else the property is a standard type
        else:
            data = property

        # Save the property to the dictionary
        dict_obj[property_name] = data
    return dict_obj



def convert_dictionary_to_ros_message(dict_obj, rosmsg_type):
    """ Recursively construct the the rosmsg from a layered dictionary """
    rosmsg_obj = rosmsg_type()

    # Loop through each property in the rosmsg
    for property_name, property_type in rosmsg_obj._fields_and_field_types.items():
        property_obj = type(getattr(rosmsg_obj, property_name))
        property_data = dict_obj[property_name]

        # If the property is a list of objects
        if 'sequence' in property_type:
            property_value = []
            item_type = property_type.replace('sequence<','').replace('>','')

            # If item type is of rosmsg object
            if '/' in item_type:
                item_pkg_name, item_class_name = item_type.split('/')
                item_obj = get_rosmsg_obj(item_pkg_name, item_class_name)
                for item_data in property_data:
                    property_value += [convert_dictionary_to_ros_message(item_data, item_obj)]

            # Else list is of standard type
            else:
                item_obj = something #TODO
                for item_data in property_data:
                    property_value += [item_type(property_data)]

        # If the property is another rosmsg object
        elif '/' in property_type:
            property_value = convert_dictionary_to_ros_message(property_data, property_obj)

        # Else the property is a standard type
        else:
            property_value = property_obj(property_data)

        # Save the property to the rosmsg
        setattr(rosmsg_obj, property_name, property_value)
    return rosmsg_obj
