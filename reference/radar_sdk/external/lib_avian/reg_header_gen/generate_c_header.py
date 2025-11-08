# ===========================================================================
# Copyright (C) 2017-2024 Infineon Technologies AG
# All rights reserved.
# ===========================================================================
#
# ===========================================================================
# This document contains proprietary information of Infineon Technologies AG.
# Passing on and copying of this document, and communication of its contents
# is not permitted without Infineon's prior written authorisation.
# ===========================================================================

# This is a python script that extracts register bitfield definition from xml
# file provided by DES team and converts it to C syntax to be included in
# driver source code.

import argparse as ap
import xml.etree.ElementTree as et
import sys
import itertools

parser = ap.ArgumentParser(description='Dump register definition in C syntax.tion dependent!')
parser.add_argument('xmlFile', metavar='XML', type=str, help='The Register XML file')
parser.add_argument('-p', '--prefix', metavar='PREFIX', nargs = 1, type=str, help='The Device Name Prefix')
args = parser.parse_args()

#-----------------------------------------------------------------------------
class Field:

    def __init__(self):
        self.name = ""
        self.offset = 0
        self.width = 0

    def getDefines(self, namePrefix):
        # both define line start like this
        commonPart = "#define "
        commonPart += namePrefix + "_" + self.name
        
        # add a define for the offset
        result = (commonPart + "_pos").ljust(50)
        result += str(self.offset) + "u\n"
        
        # add a define for the bit mask
        mask = ((1 << self.width) - 1) << self.offset
        result += (commonPart + "_msk").ljust(54)
        result += "{0:#0{1}x}".format(mask,8) + "u\n"
        
        return result

#-----------------------------------------------------------------------------
class Register:
    def __init__(self):
        self.name = ""
        self.offset = 0
        self.fields = []

    def __lt__(self, other):
        return self.offset < other.offset

    def __iter__(self):
        return iter(self.fields)

    def getBitFieldDefines(self, namePrefix):
        comment = "Fields of register " + self.name
        result = "/* " + comment + " */\n"
        result += "/* " + ("-" * len(comment)) + " */\n"
        for field in self:
            result += field.getDefines(namePrefix + "_" + self.name);
        return result

#-----------------------------------------------------------------------------
class RegisterSet:
    def __init__(self):
        self.registers = []
        self.prefix = ""

    def getRegisterDefines(self):
        result = ""
        
        for register in self.registers:
            defineLine = "#define "
            defineLine += self.prefix
            defineLine += "_REG_"
            defineLine += register.name
            defineLine = defineLine.ljust(40)
            defineLine += "{0:#0{1}x}".format(register.offset,4)
            defineLine += "u\n"
            result += defineLine
        return result

    def getBitFieldDefines(self):
        result = ""
        
        for register in self.registers:
            result += register.getBitFieldDefines(self.prefix)
            result += "\n"
        
        return result

#-----------------------------------------------------------------------------
def readXML():
    try:
        tree = et.parse(args.xmlFile)
    except Exception as details:
        sys.exit(str(details))
    
    registerSet = RegisterSet()
    root = tree.getroot()
    regMemSet = root.find('RegMemSet')
    
    for register in regMemSet.findall('RegMemElement'):
        currentRegister = Register()
        registerSet.registers.append(currentRegister)
        currentRegister.name = register.find('Name').text
        currentRegister.offset = int(register.find('Offset').text, 0)
        for field in register.findall('BitFieldElement'):
            currentField = Field()
            currentRegister.fields.append(currentField)
            currentField.name = field.find('Name').text
            currentField.offset = int(field.find('Offset').text, 0)
            currentField.width = int(field.find('Width').text, 0)
    
    
    if (args.prefix):
        registerSet.prefix = args.prefix[0]
    else :
        registerSet.prefix = root.find('Name').text
        

    return registerSet

#-----------------------------------------------------------------------------
registerSet = readXML()

print(registerSet.getRegisterDefines())

print(registerSet.getBitFieldDefines())

