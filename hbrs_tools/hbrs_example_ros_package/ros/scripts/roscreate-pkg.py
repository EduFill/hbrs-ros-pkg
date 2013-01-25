#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: roscreatepkg.py 16412 2012-02-28 18:50:18Z kwc $
# 
# modified by:  Frederik Hegger
# date:         2012-09-11
#

import roslib; roslib.load_manifest('roscreate')

NAME='roscreate-pkg'

import os
import sys
import roslib.packages
import shutil

from roscreate.core import read_template, author_name
from rospkg import on_ros_path, RosPack, ResourceNotFound

def get_templates():
    templates = {}
    templates['CMakeLists.txt'] = read_template('CMakeLists.tmpl')
    templates['manifest.xml'] = read_template('manifest.tmpl')
    templates['mainpage.dox'] = read_template('mainpage.tmpl')
    templates['Makefile'] = read_template('Makefile.tmpl')
    return templates

def instantiate_template(template, package, brief, description, author, depends):
    return template%locals()

def create_package(package, author, depends, uses_roscpp=False, uses_rospy=False):
    p = os.path.abspath(package)
    if os.path.exists(p):
        print >> sys.stderr, "%s already exists, aborting"%p
        sys.exit(1)

    os.makedirs(p)
    print "Created package directory", p

    ###### BRSU EXTENSION ###############################################################
    print "Created BRSU folder structure", p
    os.makedirs(p + "/ros")
    os.makedirs(p + "/ros/include")
    os.makedirs(p + "/ros/src")
    os.makedirs(p + "/ros/scripts")
    os.makedirs(p + "/ros/config")
    os.makedirs(p + "/ros/data")
    os.makedirs(p + "/ros/launch")
    os.makedirs(p + "/ros/nodes")
    os.makedirs(p + "/common")
    os.makedirs(p + "/common/include")
    os.makedirs(p + "/common/src")
    os.makedirs(p + "/common/scripts")
    os.makedirs(p + "/common/config")
    os.makedirs(p + "/common/data")

    # copy node and launch example from the example package and rename it
    example_pkg_dir = roslib.packages.get_pkg_dir('hbrs_example_ros_package')

    if package.startswith('raw_'):
        pkg_name_without_prefix = package[4:]
    elif package.startswith('brsu_'):
        pkg_name_without_prefix = package[5:]
    elif package.startswith('hbrs_'):
        pkg_name_without_prefix = package[5:]
    

    # copy exmaple cpp and h files
    shutil.copy2(example_pkg_dir + '/ros/nodes/ros_node_example.cpp', p + '/ros/nodes/' + pkg_name_without_prefix + '.cpp')
    shutil.copy2(example_pkg_dir + '/common/src/my_functional_class.cpp', p + '/common/src/my_functional_class.cpp')
    shutil.copy2(example_pkg_dir + '/common/include/my_functional_class.h', p + '/common/include/my_functional_class.h')

    # create launch file
    #shutil.copy2(example_pkg_dir + '/ros/launch/ros_launch_example.launch', p + '/ros/launch/' + pkg_name_without_prefix + '.launch')
    launch_file = open(p + '/ros/launch/' + pkg_name_without_prefix + '.launch', "w")
    launch_file.write("<?xml version=\"1.0\"?>\n")
    launch_file.write("<launch>\n")
    launch_file.write("    <node pkg=\"" + package + "\" type=\"" + pkg_name_without_prefix + "\" name=\"" + package + "\" output=\"screen\">\n")
    launch_file.write("        <param name=\"my_param\" type=\"double\" value=\"1.5\" />\n")
    launch_file.write("    </node>\n")
    launch_file.write("</launch>\n")
    launch_file.close()    

        
    templates = get_templates()
    for filename, template in templates.iteritems():
        contents = instantiate_template(template, package, package, package, author, depends)
        p = os.path.abspath(os.path.join(package, filename))
        with open(p, 'w') as f:
            f.write(contents.encode('utf-8'))
        print "Created package file", p


    ###### BRSU EXTENSION ###############################################################
    # Add node to the CMakeFile
    p = os.path.abspath(os.path.join(package, "CMakeLists.txt"))

    with open(p, 'a') as f:
        f.write("\n\n")
        f.write("include_directories(\n")
        f.write("   ${PROJECT_SOURCE_DIR}/common/include\n")
        f.write("   ${PROJECT_SOURCE_DIR}/ros/include\n")
        f.write(")\n\n")
        f.write("#compile the class(es) into a library\n")
        f.write("rosbuild_add_library(my_functionality_lib common/src/my_functional_class.cpp)\n\n")
        f.write("#create an executable for your ros node\n")
        f.write("rosbuild_add_executable(" + pkg_name_without_prefix + " ros/nodes/" + pkg_name_without_prefix + ".cpp)\n\n")
        f.write("#link a executable against the previously created library\n")
        f.write("target_link_libraries(" + pkg_name_without_prefix + " my_functionality_lib)\n")
        f.write("\n\n")
    
    print "\nPlease edit %s/manifest.xml to finish creating your package"%package

def roscreatepkg_main():
    from optparse import OptionParser    
    parser = OptionParser(usage="usage: %prog <package-name> [dependencies...]", prog=NAME)
    options, args = parser.parse_args()
    if not args:
        parser.error("you must specify a package name and optionally also list package dependencies")
    package = args[0]

    ###### BRSU EXTENSION ###############################################################
    if (not package.startswith('raw_')) and (not package.startswith('brsu_')) and (not package.startswith('hbrs_')):
        parser.error("illegal package name: %s\n\nPackage name must start either with 'raw_', 'brsu_' or 'hbrs_'"%package)

    if not roslib.names.is_legal_resource_base_name(package):
        parser.error("illegal package name: %s\n\nNames must start with a letter and contain only alphanumeric characters\nand underscores."%package)

    # validate dependencies and turn into XML
    depends = args[1:]
    uses_roscpp = 'roscpp' in depends
    uses_rospy = 'rospy' in depends

    ###### BRSU EXTENSION ###############################################################
    depends.append("roscpp")
    depends.append("sensor_msgs")
    depends.append("std_srvs")
    
    for d in depends:
        try:
            roslib.packages.get_pkg_dir(d)
        except roslib.packages.InvalidROSPkgException:
            print >> sys.stderr, "ERROR: dependency [%s] cannot be found"%d
            sys.exit(1)
    depends = u''.join([u'  <depend package="%s"/>\n'%d for d in depends])


    if not on_ros_path(os.getcwd()):
        print >> sys.stderr, '!'*80+"\nWARNING: current working directory is not on ROS_PACKAGE_PATH!\nPlease update your ROS_PACKAGE_PATH environment variable.\n"+'!'*80
    if type(package) == str:
        package = package.decode('utf-8')
    create_package(package, author_name(), depends, uses_roscpp=uses_roscpp, uses_rospy=uses_rospy)

if __name__ == "__main__":
    roscreatepkg_main()
