#!/usr/bin/python3
# Copyright (c) 2020, RTE (http://www.rte-france.com)
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

import getpass
import io
import os
import requests
import zipfile

def getArtifacts(workflowId):
    artifacts = []
    
    url = "https://api.github.com/repos/powsybl/powsybl-math-native/actions/runs/{0}/artifacts".format(workflowId)
    response = requests.get(url)

    if response.status_code != 200:
        raise RuntimeError("Unable to get artifacts (HTTP code: {0})".format(response.status_code))
        
    json = response.json()
    for artifact in json['artifacts']:
        artifacts.append({
            'id': artifact['id'],
            'name': artifact['name'],
            'url': artifact['archive_download_url']
        })
    
    print('Workflow {0}: {1} artifacts found...'.format(workflowId, len(artifacts)))
    
    return artifacts

def downloadArtifacts(artifacts, token):
    FOLDERS = {
        'libmath.so': 'linux_64',
        'libmath.dylib': 'osx_64',
        'math.dll': 'windows_64',
    }

    headers = {'Authorization': 'token ' + token}

    for artifact in artifacts:
        print('Downloading {0}...'.format(artifact['name']))

        response = requests.get(artifact['url'], headers=headers)

        with zipfile.ZipFile(io.BytesIO(response.content)) as zipObj:
            folder = 'target/classes/natives/' + FOLDERS[artifact['name']]
            os.makedirs(folder, exist_ok=True)
            zipObj.extract(artifact['name'], folder)

if __name__ == '__main__':
    token = input('GitHub token: ')
    workflowId = input('GitHub workflow ID: ')

    artifacts = getArtifacts(workflowId)       
    downloadArtifacts(artifacts, token)
