from setuptools import setup, find_packages
from distutils.command.build import build as DistutilsBuild
from distutils.errors import CompileError
import subprocess
import os
import sys


class BuildGazeboPlugins(DistutilsBuild):
    """This is a custom class to build the c++ Gazebo plugins during 
    installation of GymFC.
    """
    def build_plugin(self):

        plugin_dir = os.path.join(os.path.dirname(__file__), 
                                  "gymfc/envs/assets/gazebo/plugins")
        print("[gymfc] plugin directory located at {}".format(plugin_dir))
        # This is a helper script to build the plugins which can also be 
        # manually called if installing by hand with pip in edit/development 
        # mode.
        build_plugin_script = './build_plugin.sh'
        p = subprocess.Popen(build_plugin_script, cwd=plugin_dir, shell=True) 
        # Wait until the process completes
        p.wait()
        return p.returncode

    def run(self):
        print("[gymfc] starting Gazebo plugin build.")
        if self.build_plugin() != 0:
            raise CompileError("Failed to compile Gazebo plugin.")
        print("[gymfc] plugins built successfully.")
        DistutilsBuild.run(self)

setup(name='gymfc',
      version='0.1.2',
      description='A universal flight control tuning framework',
      author='William Koch',
      author_email='wfkoch [at] gmail [dot] com',
      url='https://github.com/wil3/gymfc',
      license='MIT',
      packages=find_packages(),
      package_data={'gymfc': [
          'gymfc.ini',
          'envs/assets/gazebo/models/attitude_control_training_rig/*.config',
          'envs/assets/gazebo/models/attitude_control_training_rig/*.sdf',
          'envs/assets/gazebo/worlds/*.world',
          'envs/assets/gazebo/plugins/build/*.so'
      ]},
      include_package_data=True,
      cmdclass={'build': BuildGazeboPlugins},
      install_requires=['numpy', 'protobuf', 'psutil>=5.3.0'],
)
