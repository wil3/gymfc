from setuptools import setup, find_packages
from distutils.command.build import build as DistutilsBuild
import subprocess
import os
import sys


class CustomBuild(DistutilsBuild):
    def check_cmake_installed(self):

        output = subprocess.getoutput("cmake --version")
        words = output.split()
        if (len(words) > 2 and
                words[0] == "cmake" and 
                words[1] == "version"):

            version_str = words[2]
            print ("Found cmake version ", version_str)
            version = list(map(int, version_str.split(".")))
            # According to the CMakeList this needs to be >=2.6
            if ((version[0] == 2 and version[1] >= 6) or version[0] >= 3):
                return True
        return False

    def build_plugin(self):

        plugin_dir = os.path.join(os.path.dirname(__file__), "gymfc/envs/assets/gazebo/plugins")
        print ("Plugin dir", plugin_dir)
        build_plugin_script = "./build_plugin.sh"
        p = subprocess.call([build_plugin_script], cwd=plugin_dir, shell=True) 
        print ("Done building plugin")

    def run(self):
        if self.check_cmake_installed():
            print ("Running custom install")
            self.build_plugin()
            DistutilsBuild.run(self)
        else:
            raise Exception("Cannot find valid cmake installation, required version >= 2.6.")

setup(name='gymfc',
      version='0.1.2',
      description='Environment for developing intelligent flight control systems',
      author='William Koch',
      author_email='wfkoch [at] bu [dot] edu',
      url='https://github.com/wil3/gymfc',
      license='MIT',
      packages=find_packages(),
      package_data={'gymfc': [
          "envs/assets/gazebo/models/attitude_control_training_rig/*.config",
          "envs/assets/gazebo/models/attitude_control_training_rig/*.sdf",
          "envs/assets/gazebo/worlds/*.world",
          "envs/assets/gazebo/plugins/build/*.so"
      ]},
      include_package_data=True,
      cmdclass={'build': CustomBuild},
      install_requires=['gym', 'numpy', 'protobuf', 'psutil>=5.3.0'],
)
