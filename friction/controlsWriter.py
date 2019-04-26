# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 09:33:06 2019

@author: mhmdk
"""

def controls_writer(name, val):
    f = open(name, "w")
    f.write("""<?xml version="1.0" encoding="UTF-8" ?>\n<OpenSimDocument Version="30000">\n\t<ControlSet name = "Control Set">\n\t\t<objects>\n\t\t\t<ControlLinear name = "hip_body_r">\n\t\t\t\t<is_model_control>true</is_model_control>\n\t\t\t\t<extrapolate>true</extrapolate>\n\t\t\t\t<default_min>0.02</default_min>\n\t\t\t\t<default_max>1</default_max>\n\t\t\t\t<filter_on>false</filter_on>\n\t\t\t\t<use_steps>false</use_steps>\n\t\t\t\t<x_nodes>\n\t\t\t\t\t<ControlLinearNode>\n\t\t\t\t\t\t<t>0</t>\n\t\t\t\t\t\t<value>""" + str(val) + """</value>\n\t\t\t\t\t</ControlLinearNode>\n\t\t\t\t\t<ControlLinearNode>\n\t\t\t\t\t\t<t>3</t>\n\t\t\t\t\t\t<value>""" + str(val) + """</value>\n\t\t\t\t\t</ControlLinearNode>\n\t\t\t\t</x_nodes>\n\t\t\t\t<min_nodes />\n\t\t\t\t<max_nodes />\n\t\t\t\t<kp>100</kp>\n\t\t\t\t<kv>20</kv>\n\t\t\t</ControlLinear>\n""")
    f.write("""\t\t\t<ControlLinear name = "hip_body_l">\n\t\t\t\t<is_model_control>true</is_model_control>\n\t\t\t\t<extrapolate>true</extrapolate>\n\t\t\t\t<default_min>0.02</default_min>\n\t\t\t\t<default_max>1</default_max>\n\t\t\t\t<filter_on>false</filter_on>\n\t\t\t\t<use_steps>false</use_steps>\n\t\t\t\t<x_nodes>\n\t\t\t\t\t<ControlLinearNode>\n\t\t\t\t\t\t<t>0</t>\n\t\t\t\t\t\t<value>""" + str(val) + """</value>\n\t\t\t\t\t</ControlLinearNode>\n\t\t\t\t\t<ControlLinearNode>\n\t\t\t\t\t\t<t>3</t>\n\t\t\t\t\t\t<value>""" + str(val) + """</value>\n\t\t\t\t\t</ControlLinearNode>\n\t\t\t\t</x_nodes>\n\t\t\t\t<min_nodes />\n\t\t\t\t<max_nodes />\n\t\t\t\t<kp>100</kp>\n\t\t\t\t<kv>20</kv>\n\t\t\t</ControlLinear>\n""")
    f.write("""\t\t</objects>\n\t\t<groups />\n\t</ControlSet>\n</OpenSimDocument>""")
    f.close()
    