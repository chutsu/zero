#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET

DOXYGEN_XML_PATH = "docs/xml"

def list_groups(path):
    files = []
    for f in os.listdir(path):
        file_name, file_ext = os.path.splitext(f)
        if file_name[0:5] == "group":
            files.append(os.path.join(path, f))
    return files

group_files = list_groups(DOXYGEN_XML_PATH)

xml_tree = ET.parse(group_files[3])
xml_root = xml_tree.getroot()


def xml_value(xml_element, tag_name):
    if xml_element.find(tag_name) is None:
        return None

    if xml_element.find(tag_name).text:
        return xml_element.find(tag_name).text.strip()

    return None


def xml_attrib_value(xml_element, tag_name, attrib_name):
    if xml_element.find(tag_name) is None:
        return None


    if xml_element.find(tag_name).attrib[attrib_name] is None:
        return None

    return xml_element.find(tag_name).attrib[attrib_name].strip()


def parse_detailed_params(xml_element):
    params_xml = xml_element.find("detaileddescription/para/parameterlist")
    if params_xml is None:
        return None
    if params_xml.attrib["kind"] != 'param':
        return None

    params = []
    param_key = 'parameternamelist/parametername'
    for param_xml in params_xml.findall('parameteritem'):
        entry = {}
        entry["name"] = xml_value(param_xml, param_key)
        entry["direction"] = xml_attrib_value(param_xml, 'parameternamelist/parametername', 'direction')
        entry["description"] = xml_value(param_xml, 'parameterdescription/para')
        params.append(entry)

    return params


def parse_funcs(xml_root):
    funcs = []

    for member in xml_root.iter("memberdef"):
        func = {
            "func_def": None,
            "func_brief": None,
            "func_params_detailed": None,
            "params": []
        }

        func["func_def"] = xml_value(member, 'definition')
        func["func_brief"] = xml_value(member, 'briefdescription/para')
        func["func_params_detailed"] = parse_detailed_params(member)

        for param in member.iter("param"):
            param_data = {'name': param.find("declname").text, 'elements': []}
            for x in param.find("type").iter():
                if x.text is None:
                    continue

                for el in x.text.split(' '):
                    el = el.strip()
                    if len(el):
                        param_data['elements'].append(el)
            func["params"].append(param_data)

        import pprint
        pprint.pprint(func)

        print()
        # break

parse_funcs(xml_root)

# print("\n".join())
