import ruamel.yaml  


file_name = 'STM32-for-VSCode.config.yaml'
yaml = ruamel.yaml.YAML()

with open(file_name) as fp:
    data = yaml.load(fp)

data['customMakefileRules'] = [{'command':'all','rule':'srec_cat $(BUILD_DIR)/$(TARGET).hex -Intel -crop 0x08008000 0x0801EFFC -fill 0xFF 0x08008000 0x0801EFFC -stm32 0x0801EFFC -o $(BUILD_DIR)/$(TARGET)_CRC.hex -Intel'},
{'command':'canflash','rule':'python3 ./PYTHON/CAN-FLASH.py'},{'command':'remote','rule':'python3 ./PYTHON/transfer.py'}]

with open(file_name, 'w') as fp:
    yaml.indent(mapping=1, sequence=3, offset=1)
    yaml.dump(data, fp)

# display file
with open(file_name) as fp:
    print(fp.read(), end='')

#dict_file = [{'sports' : ['soccer', 'football', 'basketball', 'cricket', 'hockey', 'table tennis']},
#{'countries' : ['Pakistan', 'USA', 'India', 'China', 'Germany', 'France', 'Spain']}]

#'STM32-for-VSCode.config.yaml'