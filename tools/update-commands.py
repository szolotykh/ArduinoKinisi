import requests
import subprocess
import os
import shutil

branch = 'main'

files = [f'https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/{branch}/commands.json']

shutil.rmtree('./', ignore_errors=True)
os.mkdir('./')
for url in files:
  response = requests.get(url)
  with open(f"./{url.split('/')[-1]}", 'wb') as f:
    f.write(response.content)

# python generator.py commands.json kinisi_commands.py --language python
subprocess.call(["python", "./generator.py", "./commands.json", "../"])