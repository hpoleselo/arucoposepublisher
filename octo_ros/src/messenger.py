#!/usr/bin/env python2.7

""" octoROS - integrating octoprint with ROS, so it's possible to control your 3d printer from your robot
This library is based in the octoprint api, more info can be found here:
http://docs.octoprint.org/en/master/api/index.html
"""


import requests

octoIP = "http://127.0.0.1"
octoPort = ":5000"
apiKey = "C721E487D74E492AAF2CB4D9E787326F"

# json key with the API Key
standardHeader = {'X-Api-Key': apiKey}

# Connection handling
def connectToPrinter():
    connectionData = {"command": "connect", "port": "/dev/ttyACM0", "baudrate": 115200, "printerProfile": "_default",
                      "save": True, "autoconnect": True}
    return requests.post(_url("connection"), json=connectionData, headers=standardHeader, timeout=5)


# File operations
def modelSelection():
    pass


def printModel(modelName):
    """" the modelname with the .gcode
    in: modelname
    return:  
    """
    printData = {'command': 'select', 'print': True}
    url = _url('files/local/{}'.format(modelName))
    return requests.post(url, json=printData, headers=standardHeader, timeout=5)


# --- SENDING COMMANDS TO THE PRINTER / JOB OPERATIONS ---

# Realize that all the functions are the same the only thing that changes is the jsonData, which is basically
# the command we're sending!

def cancelPrinting():
    # Format is: command: YOUR_COMMAND
    # And we are passing it as a JSON (with key values)
    jsonData = {'command':'cancel'}
    return requests.post(_url('job'), headers=standardHeader, timeout=5, json=jsonData)

def pausePrinting():
    jsonData = {'command':'pause', 'action':'pause'}
    return requests.post(_url('job'), headers=standardHeader, timeout=5, json=jsonData)

def resumePrinting():
    jsonData = {'command':'pause', 'action':'resume'}
    return requests.post(_url('job'), headers=standardHeader, timeout=5, json=jsonData)

# --- RETRIEVING INFORMATION FROM PRINTER ---
def progressTracking():
    response = requests.get(_url('job'), headers=standardHeader, timeout=5)
    return (response.json()['progress']['completion'])

def rateState(isPrinting, isPaused, isReadyToPrint, isCancelled):
    """ Function that rates all the states from the printer and returns just one state to send as a String
    to ROS. Short: encapsulates all states in one function. This function is expandable, you can add more 
    states to be rated."""
    if isReadyToPrint:
        finalState = "Available and ready for Printing"
    elif isPaused:
        finalState = "Printing process has been paused"
    elif isPrinting:
        finalState = "Printing"
    elif isCancelled:
        finalState = "Cancelling printing."
    else:
        finalState = "Offline"
    #elif:
    #    finalState = "Busy_Yellow"
    #elif:
    #    finalState = "Busy_Red"
    print "TESTE, TIPO DA VARIAVEL, TEM Q SER STRING"
    print type(finalState)
    return finalState

def getprinterInfo():
    response = requests.get(_url('printer'), headers=standardHeader, timeout=5)

    # Actual measurements
    tool0TempA = response.json()['temperature']['tool0']['actual']
    tool1TempA = response.json()['temperature']['tool1']['actual']
    bedTempA = response.json()['temperature']['bed']['actual']
    
    # Additional information besides what Daniel has made
    isPrinting = response.json()['state']['flags']['printing']
    isPaused = response.json()['state']['flags']['pausing']
    isReadyToPrint = response.json()['state']['flags']['operational']
    isCancelled = response.json()['state']['flags']['cancelling']
    
    # Targets
    tool0TempT = response.json()['temperature']['tool0']['target']
    tool1TempT = response.json()['temperature']['tool1']['target']
    bedTempT = response.json()['temperature']['bed']['target']

    # Call function to encapsulate all states in one
    printerState = rateState(isPrinting, isPaused, isReadyToPrint, isCancelled)

    return tool0TempA, tool1TempA, bedTempA, tool0TempT, tool1TempT, bedTempT, isPrinting, isPaused

def getFileInfo():
    """ Function to retrieve the information from the file being printed.
    to understand more, see:
    http://docs.octoprint.org/en/master/api/files.html
    """

    file_response = requests.get(_url('files'), headers=standardHeader, timeout=5)
    # We have to say the index first because as documented, the files key has a list of keys, not key of keys...
    # and convert it to str because we're getting a unicode type (ROS will complain about it later.)
    fileName = str(file_response.json()['files'][0]['name'])
    fileSize = file_response.json()['files'][0]['size']
    # In minutes
    estimatedTime = file_response.json()['files'][0]['gcodeAnalysis']['estimatedPrintTime']
    # In hours, TODO: convert the rest to minutes
    estimatedTime = round(estimatedTime/60,2)
    return fileName, estimatedTime, fileSize

def getTimeLapse():
    # Not working yet. Returning <Response [200]>
    gtl_response = requests.get(_url('timelapse'), headers=standardHeader, timeout=5)
    return gtl_response

def _url(path):
    """ Function to pass the URL """
    octoAddress = octoIP + octoPort + '/api/'
    return octoAddress + path
