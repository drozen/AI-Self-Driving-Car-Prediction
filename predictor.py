#!/usr/bin/env python
__author__ = 'agmadi'

import sys, os
import getopt
from rangeFinder import RangeFinder
from EKF import EKF
from pickle import dump, load

def usage():
    """
    Print usage information
    :rtype : None
    """
    print '''\
Usage: %s [OPTION]...
Predict Hexbug Motion given coordinates in file
Input will be read from a file called testing_video-centroid_data by default
Output will be writted to a file called prediction.txt by default
A file called 'learned' will be created that contains any learned data
  This file will be used in subsequent runs

  -c, --clean            Do not use previously learned data
  -d, --debug            Write debug output while running
  -f, --file=FILE        Read coordinates from FILE
                         Defaults to testing_video-centroid_data
  -h, --help             Print this help screen and exit
  -l, --learn            Learn from data and create a learned file
  -o, --output=FILE      Write output to FILE
                         Defaults to prediction.txt
  -p, --predict=FRAMES   Predict FRAMES number of frames
                         Defaults to 60
  -v, --visualize        Show a visual of the predictions after processing
                         Includes the last 120 points of the original data
                         (or all of it if less than 120 exists)
  -y, --yaw=YAW_LIMIT    Limit yaw noise
                         Defaults to 0.015
                         Set to zero to turn off
''' % ( os.path.basename( sys.argv[0] ) )

def gatherOpts():
    """
    Parse command line options
    :rtype : ( string, string, int, float, bool, bool, bool )
    :return: input file path, output file path, number of frames to predict, yaw limit, visualization boolean, debug boolean, clean boolean, learn boolean
    """
    # set up defaults
    learn = False
    clean = False
    debug = False
    InputFile = 'testing_video-centroid_data'
    OutputFile = 'prediction.txt'
    PredictFrames = 60
    Visualize = False
    YawLimit = 0.015

    try:
        opts, args = getopt.getopt( sys.argv[1:], "cldhf:o:p:y:v", [ "clean", "learn", "debug", "help", "file=", "output=", "predict=", "yaw=", "visualize" ] )
    except Exception, e:
        print 'ERROR: %s\n' % e
        usage()
        sys.exit(1)

    for option, value in opts:
        if option in [ '-f', '--file' ]:
            InputFile = value
        elif option in [ '-d', '--debug' ]:
            debug = True
        elif option in [ '-c', '--clean' ]:
            clean = True
        elif option in [ '-l', '--learn' ]:
            learn = True
        elif option in [ '-h', '--help' ]:
            usage()
            sys.exit(0)
        elif option in [ '-o', '--output' ]:
            OutputFile = value
        elif option in [ '-p', '--predict' ]:
            try:
                PredictFrames = int( value )
            except:
                print 'ERROR: Number of frames to predict must be an integer'
                sys.exit(1)
        elif option in [ '-y', '--yaw' ]:
            try:
                YawLimit = float( value )
            except:
                print 'ERROR: Yaw Limit must be a decimal'
                sys.exit(1)
        elif option in [ '-v', '--visualize' ]:
            Visualize = True

    return ( InputFile, OutputFile, PredictFrames, YawLimit, Visualize, debug, clean, learn )

def readDataFile( fileName ):
    """
    Read data file and return it as a Python object
    :rtype : list
    :type fileName: str
    :return: list of coordinates
    """
    f = open( fileName )
    allLines = f.read().replace( '\n', '' )
    return eval( allLines )

def analyzeData( rawData, dataMinX, dataMinY, dataMaxX, dataMaxY ):
    """
    Analyze coordinate data read from file and add information needed by EKF.
    Gather minimums and maximums.
    :param dataMinX: float
    :param dataMinY: float
    :param dataMaxX: float
    :param dataMaxY: float
    :rtype : ( RangeFinder, list, float, float, float, float )
    :type rawData: list
    :return: a RangeFinder object for the raw data; the raw data extended with heading, velocity, and yaw rate;
    minimum coordinates; maximum coordinates.
    """
    r = RangeFinder(rawData, dataMinX, dataMinY, dataMaxX, dataMaxY)
    r.updateAll()
    extendedData = r.getNewData()
    dataMinX, dataMinY, dataMaxX, dataMaxY = r.getRange()
    return ( r, extendedData, dataMinX, dataMinY, dataMaxX, dataMaxY )

def main():
    """
    Main function: get command line args, read data, call EKF, and show visualization.
    :rtype : None
    """
    InputFile, OutputFile, PredictFrames, YawLimit, Visualize, Debug, Clean, Learn = gatherOpts()

    if not os.access( InputFile, os.R_OK ):
        print 'ERROR: %s is not accessible' % InputFile
        sys.exit(1)
    if PredictFrames < 1:
        print 'ERROR: Number of frames to predict must be greater than 0'
        sys.exit(1)

    if Debug:
        print 'InputFile: ', InputFile
        print 'OutputFile: ', OutputFile
        print 'PredictFrames: ', PredictFrames
        print 'YawLimit: ', YawLimit
        print 'Visualize: ', Visualize
        print 'Clean:', Clean
        print 'Learn:', Learn

    # read in any prelearned data, including where the walls are and the covariance.
    dataMinX = dataMinY = dataMaxX = dataMaxY = 0
    pvector = None
    if not Clean and os.access( 'learned', os.R_OK ):
        f = open( 'learned' )
        ( minx, miny, maxx, maxy, pvector ) = load( f )
        dataMinX = float( minx )
        dataMinY = float( miny )
        dataMaxX = float( maxx )
        dataMaxY = float( maxy )
        f.close()

    rawData = readDataFile( InputFile )
    ranger, extendedData, dataMinX, dataMinY, dataMaxX, dataMaxY = analyzeData( rawData, dataMinX, dataMinY, dataMaxX, dataMaxY )
    EKFilter = EKF( ranger, extendedData, dataMinX, dataMinY, dataMaxX, dataMaxY, PredictFrames, YawLimit, Debug )

    # if there is a prelearned covariance, then use it
    if pvector is not None:
        EKFilter.PLearned = pvector

    f = open( OutputFile, 'w' )
    for i in EKFilter.runFilter():
        f.write( "%d,%d\n" % ( int(i[0]), int(i[1]) ) )
    f.close()

    if Learn:
        f = open( 'learned', 'w' )
        dump( ( dataMinX, dataMinY, dataMaxX, dataMaxY, EKFilter.PLearned ), f )
        f.close()

    if Visualize:
        EKFilter.visualize()

if __name__ == '__main__':
    main()
