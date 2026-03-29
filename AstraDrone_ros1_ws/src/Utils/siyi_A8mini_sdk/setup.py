
from setuptools import setup, find_packages

setup(
    name='siyiA8mini',      # 包名
    version='0.1.0',               # 版本号
    packages=find_packages(),      # 自动找到项目中的所有包
    install_requires=['pynput'],     #依赖项
    author='张鹏程',            # 作者信息
    author_email='zhangpengcheng@sjtu.edu.cn',  # 作者邮箱
    description='A python SDK for SIYI A8mini camera',  # 包的描述
    long_description=open('README.md',encoding='UTF-8').read(),    
    long_description_content_type='text/markdown', 
    url='https://github.com/Percylevent/SIYISDK',  
    classifiers=[
        'Programming Language :: Python :: 3',  
        'License :: OSI Approved :: MIT License',  
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.8',  
)
