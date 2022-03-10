#!python
# encoding: utf-8
import os
from zipfile import ZipFile,ZIP_DEFLATED
from glob import glob
from os.path import basename
import tomli

os.system("cargo build --release")
os.system("del dist /s /q")
os.system("poetry build -f sdist")
os.system("python generate_docs.py")

with open("../Cargo.toml", "rb") as f:
    toml_dict = tomli.load(f)
version = toml_dict["package"]["version"]

with ZipFile(f'dist/ktanks_{version}.zip', 'w',compression=ZIP_DEFLATED,compresslevel=9) as zipObj2:
    #for x in glob("dist/*"):
    #    zipObj2.write(x,arcname=basename(x))
    for x in glob("../target/release/*.exe"):
        zipObj2.write(x,arcname=basename(x))
    
   # zipObj2.write("build/rinoh/ktanks.pdf",arcname="ktanks_python_reference.pdf")
   # zipObj2.write("../README.md",arcname="README.md")
    
with ZipFile(f'dist/examples_{version}.zip', 'w',compression=ZIP_DEFLATED,compresslevel=9) as zipObj2:
    zipObj2.write("multi_launcher.py")
    zipObj2.write("README.md")
    for x in glob("examples/*.*"):
        zipObj2.write(x,arcname="examples/"+basename(x))