import sys
import shutil
import os

try:
  root_dir = sys.argv[1]
  destination = sys.argv[2]
  print(root_dir, destination)

  for dirpath, _, files in os.walk(root_dir):
    for file in files:
      if os.path.splitext(file)[1] in ['.c', '.s'] and \
              os.path.normpath(dirpath) != os.path.normpath(destination):
        shutil.copy(os.path.join(dirpath, file), destination)

except:
  raise Exception('Error copying files')

'''
for arg in sys.argv:
  if os.path.basename(__file__) not in arg:
    if os.path.isfile(arg):
      source_files += [arg]
    elif os.path.isdir(arg):
      destination = arg

if len(source_files) > 0 and destination != '':
  for file in source_files:
    shutil.copy(file, destination)
else:
'''