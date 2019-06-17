#!/usr/bin/env python3

import urllib.request
import tarfile
import sys

version = sys.argv[1]
archiveName = 'SuiteSparse-' + version + '.tar.gz'
url = 'http://faculty.cse.tamu.edu/davis/SuiteSparse/' + archiveName

print('Downloading archive %s...' % url)

urllib.request.urlretrieve(url, archiveName)  

print('Extracting files')

with tarfile.open(archiveName, "r:gz") as tar:
    tar.extractall()

