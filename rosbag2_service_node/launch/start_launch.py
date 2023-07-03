"""
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Bernat Gaston
   Contact: support.idi@ageve.net
"""

from ageve_utils.launch.start import QuickClass
from ageve_utils.general.yaml import getNamespace
from ageve_utils.general.system import getPkgName
    
def generate_launch_description():

   Package = getPkgName(__file__)
   namespace = getNamespace(Package) # En caso de usar sin bringup utilizar getNamespace(Package, standalone=True)
   launcher = QuickClass(Package, namespace)
   nodes = launcher.getPkgNodes("me00_bringup")
   for node in nodes:
      nodes_dic = launcher.set_dictionary(package=Package, ros_name=node, log="info")
   return launcher.launch_nodes(nodes_dic)

    