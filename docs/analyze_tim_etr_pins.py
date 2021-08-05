#!/usr/bin/env python
"""
Simple python 3 script to collect information about STM32 timer ETR pins.

Usage:
1. Install python 3.6 or higher
2. Download https://github.com/STMicroelectronics/STM32_open_pin_data repository into current folder
3. Run this script to print most common ETR pins (GPIO ports "A" and "B", timers "TIM1", "TIM2", "TIM3") of STM32 MCUs.
"""

import logging
import operator
import os
import os.path
import re
import xml.etree.ElementTree as ET

logger = logging.getLogger(__name__)

_NAMESPACE_RE = re.compile(r'\bxmlns="[^"]+"')


def _read_xml_without_namespaces(xml_file):
    with open(xml_file, encoding='utf-8') as f:
        data = f.read()
    # cleanup namespace. It isn't correct solution, but is the simplest one
    data = _NAMESPACE_RE.sub('', data, 1)
    return ET.fromstring(data)


_PIN_NAME_RE = re.compile(r'^P(?P<port>[A-Z])(?P<pin>\d{1,2})\b')

_TIM_ETR_RE = re.compile(r'^TIM(?P<tim_no>\d+)_ETR$')

_STM32F1_AF_MAP = {
    "NONE": "0",
    "SPI1_ENABLE": "1",
    "I2C1_ENABLE": "2",
    "USART1_ENABLE": "3",
    "USART3_PARTIAL": "5",
    "TIM1_PARTIAL": "6",
    "TIM3_PARTIAL": "7",
    "TIM2_ENABLE": "8",
    "TIM2_PARTIAL_1": "8",
    "TIM2_PARTIAL_2": "8",
    "TIM3_ENABLE": "9",
    "CAN1_2": "10",
}
_STM32F1_AF_NAME_RE = re.compile(r'REMAP_(?P<af_name>[A-Z0-9_]+)$')
_STM32_COMMON_AF_NAME_RE = re.compile(r'^GPIO_AF(?P<af_no>\d+)_')


def _load_tim_etr_pins(mcu_elem):
    # get ETR pins
    etr_pin_map = []

    for pin in mcu_elem.findall('Pin'):
        pin_name_stm = pin.attrib.get('Name', '')
        m = _PIN_NAME_RE.match(pin_name_stm)
        if m is None:
            continue
        for signal in pin.findall('Signal'):
            signal_name = signal.attrib.get('Name', '')
            m = _TIM_ETR_RE.match(signal_name)
            if m is None:
                continue
            etr_pin_map.append({
                'stm_pin': pin_name_stm,
                'stm_signal': signal_name
            })

    return etr_pin_map


def _add_tim_etr_af(mcu_elem, gpio_elem, etr_pin_map):
    mcu_family = mcu_elem.attrib['Family'].strip().upper()
    gpio_functions = {elem.attrib['Name']: elem for elem in gpio_elem.findall('GPIO_Pin')}
    for pin_info in etr_pin_map:
        gpio_pin_info = f"{pin_info['stm_pin']}/{pin_info['stm_signal']} (GPIO-{gpio_elem.attrib['Version']}_Models.xml)"

        gpio_function_elem = gpio_functions[pin_info['stm_pin']]
        signal_info_elem = next(filter(
            lambda elem: elem.attrib.get('Name') == pin_info['stm_signal'],
            gpio_function_elem.findall("PinSignal")
        ), None)

        if signal_info_elem is None:
            af_num = 'GPIO_AF_NONE'
        elif mcu_family == "STM32F1":

            remap_block_elem = signal_info_elem.find('RemapBlock')
            if remap_block_elem is None:
                raise ValueError(f"Cannot find alternative function for {gpio_pin_info}")
            remap_value_elem = remap_block_elem.find('.//PossibleValue')
            remap_value = '' if remap_value_elem is None else remap_value_elem.text.strip()
            if not remap_value:
                af_num = 0
            else:
                m = _STM32F1_AF_NAME_RE.search(remap_value)
                if m is None or m.group('af_name') not in _STM32F1_AF_MAP:
                    # drop such function
                    af_num = None
                else:
                    af_num = _STM32F1_AF_MAP[m.group('af_name')]
        else:
            af_value_elem = signal_info_elem.find('.//PossibleValue')
            if af_value_elem is None:
                raise ValueError(f"Cannot find alternative function for {gpio_pin_info}")
            af_num = af_value_elem.text.strip()

        pin_info['af_num'] = af_num

    # cleanup records without af_num
    etr_pin_map[:] = [pin_info for pin_info in etr_pin_map if pin_info['af_num'] is not None]


def _add_time_etr_mbed_names(etr_pin_map):
    # convert pin/signal names to mbed
    for pin_info in etr_pin_map:
        m = _PIN_NAME_RE.match(pin_info['stm_pin'])
        if m is None:
            raise ValueError(f"Invalid pin name: {pin_info['stm_pin']}")
        pin_info['mbed_pin'] = f'P{m.group("port")}_{m.group("pin")}'
        pin_info['mbed_pin_id'] = (m.group("port"), int(m.group('pin')))
        m = _TIM_ETR_RE.match(pin_info['stm_signal'])
        if m is None:
            raise ValueError(f"Invalid signal name: {pin_info['stm_signal']}")
        pin_info['mbed_signal'] = f'PWM_{m.group("tim_no")}'

    # sort map by pin id
    etr_pin_map[:] = sorted(etr_pin_map, key=operator.itemgetter('mbed_pin_id'))


def _parse_tim_etr_pins(mcu_file):
    mcu_elem = _read_xml_without_namespaces(mcu_file)
    mcu_family = mcu_elem.attrib['Family'].strip().upper()
    mcu_line = mcu_elem.attrib['Line'].strip().upper()
    mcu_ref_name = mcu_elem.attrib['RefName'].strip().upper()

    # open corresponding GPIO file
    gpio_ip_elem = mcu_elem.find("IP[@Name='GPIO']")
    if gpio_ip_elem is None:
        raise ValueError(f"Cannot find GPIO file for \"{mcu_file}\" MCU")
    gpio_file = os.path.abspath(os.path.join(mcu_file, '..', 'IP', f'GPIO-{gpio_ip_elem.attrib["Version"]}_Modes.xml'))
    if not os.path.exists(gpio_file):
        raise ValueError(f"Cannot find GPIO file \"{gpio_file}\" for \"{mcu_file}\" MCU")
    gpio_elem = _read_xml_without_namespaces(gpio_file)

    try:
        # load ETR pins
        etr_pin_map = _load_tim_etr_pins(mcu_elem)
        # add alternative functions
        _add_tim_etr_af(mcu_elem, gpio_elem, etr_pin_map)
        # add mbed names
        _add_time_etr_mbed_names(etr_pin_map)
    except Exception as e:
        raise ValueError(f"Fail to parse MCU \"{mcu_ref_name}\"") from e

    return {
        'mcu_family': mcu_family,
        'mcu_line': mcu_line,
        'mcu_ref_name': mcu_ref_name,
        'etr_pins': etr_pin_map
    }


_BASE_TIMS = {'PWM_1', 'PWM_2', 'PWM_3'}
_BASE_PORTS = {'A', 'B'}


def _etr_filter(etr_pin):
    if etr_pin['mbed_signal'] not in _BASE_TIMS:
        return False
    port = etr_pin['mbed_pin'][1]
    if port not in _BASE_PORTS:
        return False
    return True


def main():
    logging.basicConfig(level=logging.INFO)

    stm32_open_pin_data_dir = os.path.join(os.path.dirname('__file__'), 'STM32_open_pin_data')
    if not os.path.exists(stm32_open_pin_data_dir):
        raise ValueError(f"Cannot find STM32_open_pin_data data folder: {stm32_open_pin_data_dir}")

    logger.info("Parse MCU files")
    mcus_etrs = {}
    for dir_entry in os.scandir(os.path.join(stm32_open_pin_data_dir, 'mcu')):
        if not dir_entry.is_file():
            continue
        if not dir_entry.name.endswith('.xml'):
            continue
        logger.info(f"Process file {dir_entry.name}")
        mcus_etrs[dir_entry.name] = _parse_tim_etr_pins(dir_entry.path)
    logger.info("MCU files has been parsed")
    logger.info("Get common ETR outputs")

    mcu_filenames = sorted(mcus_etrs)

    for mcu_filename in mcu_filenames:
        mcu_etrs = mcus_etrs[mcu_filename]

        base_etr_outs = list(filter(_etr_filter, mcu_etrs['etr_pins']))
        base_etr_outs_str = ' - '.join(
            f'{etr_pin["mbed_pin"]}/{etr_pin["mbed_signal"]}/{etr_pin["af_num"]}'
            for etr_pin in base_etr_outs
        )

        print(f"{mcu_filename:32}: {base_etr_outs_str}")


if __name__ == '__main__':
    main()
