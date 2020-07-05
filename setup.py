from setuptools import setup, find_packages

setup(name='gym_pybullet_kuka',
      version='0.0.1',
      install_requires=['gym>=0.11.0', 'pybullet>=1.9.7'],
      packages=find_packages(include=['gym_pybullet_kuka', 'gym_pybullet_kuka.*']),
      package_data={'gym_pybullert_kuka':['assets/*']},
      author='Elias Hanna',
      author_email='hanna@isir.upmc.fr'
)
