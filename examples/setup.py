from setuptools import setup, find_packages
setup(name='gymfc_nf',
      version='1.0.0',
      description='Gym for synthesizing neuro-controller for Neuroflight',
      author='William Koch',
      author_email='wfkoch [at] gmail [dot] com',
      license='MIT',
      packages=find_packages(),
      install_requires=['matplotlib', 'gym', 'numpy'],
)
