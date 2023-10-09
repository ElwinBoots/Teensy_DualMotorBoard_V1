# -*- coding: utf-8 -*-
"""
Created on Mon May  9 22:37:25 2022

@author: Elwin
"""

# import pyclibrary

# p = pyclibrary.c_parser.CParser(files="defines.h"  )
# p.process_all()

# variables = p.defs['variables']
# structs = p.defs['structs']

# structs['motor_total_t']['members'][0]


# for a in variables:
#     print(a)
# for a in variables.keys():
#     print(variables[a][1][0])

import pycstruct

definitions = pycstruct.parse_file('defines.h')

# with open('defines.h', 'rb') as f:
#     inbytes = f.read()

# Dictionary representation
# instance = definitions['motor_total_t'].deserialize(inbytes)
instance = definitions['motor_total_t'].create_empty_data()

print('\n\
enum type {\n\
  TYPE_INT8,\n\
  TYPE_UINT8,\n\
  TYPE_INT16,\n\
  TYPE_UINT16,\n\
  TYPE_INT32,\n\
  TYPE_UINT32,\n\
  TYPE_INT64,\n\
  TYPE_UINT64,\n\
  TYPE_FLOAT32,\n\
  TYPE_FLOAT64\n\
};')

print('\n\
typedef struct trace_t{ \n\
  const char* names[500]; \n\
  uint8_t  all_types[500]; \n\
  uint8_t* all_pointers[500]; \n\
  uint32_t  all_lengths[500]; \n\
  uint8_t*  pointers[50]; \n\
  uint32_t  lengths[50]; \n\
  uint32_t n_to_send;\n\
  bool send_all;\n\
} trace_t; \n\
\n\
trace_t trace = \n\
{ \n\
  {')

motors = ['motor']
for motor in motors:
    for a in instance:
        try:
            for b in instance[a]:
                print( '	"' + motor + '.' + a + '.' + b +  '",' )
        except TypeError:
                print(  '	"' + motor + '.' + a +  '",')
print('  }\n\
,\n\
  {')        

a = definitions['motor_total_t'][0].dtype()

b = a[0]
names = b['names'];

for motor in motors:
    for c in b['formats']:
        if isinstance(c, tuple):
            c = c[0]  
        if isinstance(c, str):
            size = int(c[2])*8;
            # print( f'	 "{d}",' )
            if c[1] == 'f':
                tp = 'FLOAT'
            if c[1] == 'i':
                tp = 'INT'
            if c[1] == 'u':
                tp = 'UINT'
            print( f'	 TYPE_{tp}{size},' )
        else:
            for d in c['formats']:
                if isinstance(d, tuple):
                    d = d[0]
                size = int(d[2])*8;
                # print( f'	 "{d}",' )
                if d[1] == 'f':
                    tp = 'FLOAT'
                if d[1] == 'i':
                    tp = 'INT'
                if d[1] == 'u':
                    tp = 'UINT'
                print( f'	 TYPE_{tp}{size},' )

print('  }\n\
,\n\
  {')  
                
for motor in motors:
    for a in instance:
        try:
            for b in instance[a]:
                print( '	(uint8_t*)&' + motor + '.' + a + '.' + b + ',')
        except TypeError:
                print(  '	(uint8_t*)&' + motor + '.' + a  + ',')     
print('  }\n\
,\n\
  {')      
                
for motor in motors:
    for a in instance:
        try:
            for b in instance[a]:
                print( '	sizeof(' + motor + '.' + a + '.' + b + '),')
        except TypeError:
                print( '	sizeof(' + motor + '.' + a  + '),')     
print('  }\n\
};')         

print('\n\
uint8_t signaltomemberlength( uint32_t isignal ) {\n\
  switch ( trace.all_types[isignal] ) {\n\
    case TYPE_INT8:    return(1);\n\
    case TYPE_UINT8:   return(1);\n\
    case TYPE_INT16:   return(2);\n\
    case TYPE_UINT16:  return(2);\n\
    case TYPE_INT32:   return(4);\n\
    case TYPE_UINT32:  return(4);\n\
    case TYPE_INT64:   return(8);\n\
    case TYPE_UINT64:  return(8);\n\
    case TYPE_FLOAT32: return(4);\n\
    case TYPE_FLOAT64: return(8);\n\
    default: return(1);\n\
  }\n\
}\n')

