import yaml

def load_yaml_file(config_file_name):
    file_handle = open(config_file_name, 'r')
    data = yaml.load(file_handle)
    file_handle.close()
    return data
