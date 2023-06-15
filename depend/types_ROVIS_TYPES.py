import os
import json
from depend.global_config import cfg
CFG = cfg

# Has to point to the ROVIS_TYPES.h file in the RovisVision repository
rovis_types_header = os.path.join(os.path.dirname(__file__), CFG.ROVIS_VISION.TYPES_FILE)

ROVIS_FILTER_TYPE_ENUM_NAME = "ROVIS_FILTER_TYPE"
ROVIS_DATA_TYPE_ENUM_NAME = "ROVIS_DATA_TYPE"

conf_file = CFG.ROVIS_VISION.CONF_FILE


def get_datatypes_as_dict(rovis_types_path, enum_name):
    found_filter_type_enum = False
    filter_type_enum_lines = list()
    with open(rovis_types_path, "r") as f:
        while True:
            line = f.readline()
            if len(line) == 0:
                break
            if "enum" in line and enum_name in line:
                found_filter_type_enum = True
            if found_filter_type_enum is True:
                if "}" in line and ";" in line:
                    filter_type_enum_lines.append(line.strip("\n"))
                    break
            if found_filter_type_enum is True:
                filter_type_enum_lines.append(line.strip("\n"))

    # Remove useless lines (comments, empty lines, brackets, semicolons
    useful_idx = list()
    block_comment = False
    for idx, l in enumerate(filter_type_enum_lines):
        if "".join(l.split()).startswith("//"):
            continue
        if "".join(l.split()).startswith("/*"):
            block_comment = True
        if block_comment is True and "".join(l.split()).endswith("*/"):
            block_comment = False
        if block_comment is True:
            continue
        if "=" in l:
            useful_idx.append(idx)

    filter_type_enum_lines = [filter_type_enum_lines[i] for i in useful_idx]
    filter_type_enum_lines = [" ".join(l.split()).split(",")[0] for l in filter_type_enum_lines]

    data_dict = dict()
    for l in filter_type_enum_lines:
        try:
            data_dict[l.split("=")[0].strip()] = int(l.split("=")[1].strip())
        except IndexError:
            return dict()

    return data_dict


class RovisFilterType:
    def __init__(self):
        """
        Instantiation not needed
        """
        raise NotImplementedError


class RovisDataType:
    def __init__(self):
        """
        Instantiation not needed
        """
        raise NotImplementedError


if os.path.exists(conf_file):
    # print("Info: Found Rovis configuration file (JSON) {}".format(
    #     conf_file
    # ))
    with open(conf_file, "r") as conf:
        conf_json = json.load(conf)
    for data_type in conf_json["RovisDataTypes"]:
        setattr(RovisDataType, data_type["name"], data_type["value"])

    for filter_type in conf_json["RovisFilterTypes"]:
        setattr(RovisFilterType, filter_type["name"], filter_type["value"])

else:
    print("Warning: Did not find configuration json file. Using deprecated ROVIS_TYPES.H")
    if not os.path.exists(rovis_types_header):
        print("ROVIS_TYPES.h: {0} not found. Exiting...".format(rovis_types_header))
        exit(-1)

    d = get_datatypes_as_dict(rovis_types_header, ROVIS_FILTER_TYPE_ENUM_NAME)
    for k in d:
        setattr(RovisFilterType, k, d[k])

    d2 = get_datatypes_as_dict(rovis_types_header, ROVIS_DATA_TYPE_ENUM_NAME)
    for k in d2:
        setattr(RovisDataType, k, d2[k])
