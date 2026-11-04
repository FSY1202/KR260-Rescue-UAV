from setuptools import setup
from glob import glob

setup(name='kr260_ground_station',version='0.1.0',
      packages=['kr260_ground_station'],
      data_files=[('share/ament_index/resource_index/packages',['resource/kr260_ground_station']),
                  ('share/kr260_ground_station',['package.xml']),
                  ('share/kr260_ground_station/config',glob('config/*.yaml')),
                  ('share/kr260_ground_station/launch',glob('launch/*.py'))],
      install_requires=['setuptools'],zip_safe=True,
      entry_points={'console_scripts':['telemetry_sender = kr260_ground_station.ros_sender:main']})
