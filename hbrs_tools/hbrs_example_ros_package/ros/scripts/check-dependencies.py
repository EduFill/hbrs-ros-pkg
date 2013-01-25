#!/usr/bin/python

import sys
import os, fnmatch
import re
import roslib
from os.path import basename

def find_files(directory, pattern_list):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            for pattern in pattern_list:
                if fnmatch.fnmatch(basename, pattern):
                    filename = os.path.join(root, basename)
                    yield filename


def get_pkg_names_used_in_launch_file(pkg_name):

    dependency_list = []

    try:
        pkg_location = roslib.packages.get_pkg_dir(pkg_name)
    except Exception, e:
        return -1;

    # find recursivly all launch files from the given location
    for filename in find_files(pkg_location, ['*.launch', '*.xml', '*.urdf', '*.xacro']):
        
        if basename(filename) == "manifest.xml" or basename(filename) == "stack.xml":
            continue

        #open file
        f = file(filename)
        
        # go through all lines
        for line in f:
            # look for find tags. usually in <include>
            if 'find' in line:
                #extract package name
                dependency_name = re.search(r'.*find\s(\w+)\).*', line, re.M|re.I).group(1)
                dependency_list.append(dependency_name)
            # look for pkg tages used in <node>
            elif 'pkg' in line:
                #extract package name              
                search_result = re.search(r'.*pkg=\"(\w+)\".*', line, re.M|re.I)
                if search_result == None:
                    continue

                dependency_name = search_result.group(1)
                dependency_list.append(dependency_name)

    #remove duplicates
    return list(set(dependency_list))


def print_dependency(dependency_name):
    print "  <depend package=\"" + str(dependency_name) + "\"/>"


def cross_check_with_manifest(pkg_name, launch_dep_list):

    pkg_location = roslib.packages.get_pkg_dir(pkg_name)
    dep_file = str("manifest.xml")
    manifest_file_location = str(pkg_location) + "/" + dep_file

    print "manifest: ", manifest_file_location

    #check if file exists
    if not os.path.exists(manifest_file_location):
        print "ERROR: " + dep_file + " does not exist in package " + pkg_name
        print "Create a " + manifest_file_location + " and add the following dependencies:\n"
        for dep_name in launch_dep_list:
            print_dependency(dep_name)        
        return

    f = file(manifest_file_location)

    print "\nDependencies which might not be used and could be removed\n"
    
    # get deps of manifest
    manifest_dep_list = []
    for line in f:
        # look for depend tags
        if 'depend' in line:
            #extract package name                
            search_result = re.search(r'.*package=\"(\w+)\".*', line, re.M|re.I)

            if search_result == None:
                continue

            manifest_dependency_name = search_result.group(1)

            if not manifest_dependency_name in launch_dep_list:
                print_dependency(manifest_dependency_name)

            manifest_dep_list.append(manifest_dependency_name)
            
    print "\n\nDependencies which are missing:\n"

    for launch_dep in launch_dep_list:
        if not launch_dep in manifest_dep_list and launch_dep != pkg_name:
            print_dependency(launch_dep)
            



if __name__ == '__main__':

    if len(sys.argv) <= 1:
        print "ERROR: you have to pass at least one package name as argument"
        exit(0)

    sys.argv.pop(0)

    for arg in sys.argv:
        print '\n###################################################################'

        dep_list = get_pkg_names_used_in_launch_file(arg)
        
        if dep_list == -1:
            print "ERROR: package '" + arg + "' does not exist."
            continue

        print "PACKAGE: ", arg
        #print "DEPENDENCIES: ", dep_list 

        cross_check_with_manifest(arg, dep_list)
        
        print '\n###################################################################'

