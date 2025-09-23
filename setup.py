from setuptools import setup
import os

package_name = 'emotion_speech'

# Find all Python files (including config.py) in your package
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            if filename.endswith('.py'):
                paths.append(os.path.join('..', path, filename))
    return paths

extra_files = package_files(package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Qaim',
    maintainer_email='qaimbaaden@email.com',
    description='Emotion-aware robot nodes for Turtlebot3 FYP',
    license='Apache License 2.0',
    tests_require=['pytest'],
    package_data={
        # Include all .py files, especially config.py!
        package_name: [os.path.basename(f) for f in extra_files]
    },
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'memory_node = emotion_speech.memory_node:main',
            'pnnx2 = emotion_speech.pnnx2:main',
            'pnnx_to_emotion2_node = emotion_speech.pnnx_to_emotion2_node:main',
            'speech_to_text_node = emotion_speech.speech_to_text_node:main',
            'gemini_node = emotion_speech.gemini_node:main',
            'face_display_node = emotion_speech.face_display_node:main',
            'face_ctrl2_cv = emotion_speech.face_ctrl2_cv:main',
            'motion_ctrl2 = emotion_speech.motion_ctrl2:main',
            'face_publisher = emotion_speech.face_publisher:main',
            'pnnx2_dummy = emotion_speech.pnnx2_dummy:main',
            
        ],
    },
)
