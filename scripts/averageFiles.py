#!/usr/bin/python

import os, os.path
import csv
from operator import add

A = os.listdir('.');

lenA = len(A);

i = 0;
navFiles = [];
vidFiles = [];
while i < lenA:
  if(A[i].startswith("videoDelays")):
    vidFiles.append(A[i]);
  elif(A[i].startswith("NavdataDelays")):
    navFiles.append(A[i]);
  i+=1;

print "Processing ",
print len(navFiles),
print " Navdata files and ",
print len(vidFiles),
print " Video Files"

if(len(navFiles) != len(vidFiles)):
  print "mismatching no. of vidFiles and navFiles.. please check current working directory.."
  exit();

i = 0;


totalNavAvg = [0]*60;
totalVidAvg = [0]*60;


while i < len(navFiles):
  navF = open(navFiles[i], 'rt');
  vidF = open(vidFiles[i], 'rt');
  navDelays = [];
  vidDelays = [];

  #Reading the nav and vid files
  navReader = csv.reader(navF);
  vidReader = csv.reader(navF);
  #skip the headers
  next(navReader, None)
  next(vidReader, None)
  navCount = 0;
  vidCount = 0;

  #Extracting first 12000 for nav and 1800 for vid
  for row in navReader:
    navDelays.append(row[2]);
    navCount+=1;
    if(navCount >= 12000):
      break;
  for row in vidReader:
    vidDelays.append(row[2]);
    vidCount+=1;
    if(vidCount >= 1800):
      break;
  navF.close();
  vidF.close();
  navAvgDelays = [];
  vidAvgDelays = [];
  idx = 0;

  #Averaging over 200 navdata
  while(idx < 12000):
    totDelay = 0;
    for num in range(idx, idx + 200):
      totDelay += float(navDelays[num]);
    navAvgDelays.append(totDelay/200);
    idx += 200;
  idx = 0;

  #Averaging over 30 vidData
  while(idx < 1800):
    totDelay = 0;
    for num in range(idx, idx + 30):
      totDelay += float(vidDelays[num]);
    vidAvgDelays.append(totDelay/30);
    idx += 30;
  #print len(navAvgDelays);
  #print len(vidAvgDelays);

  #Adding to the total average Delays
  totalNavAvg = map(add, totalNavAvg, navAvgDelays);
  totalVidAvg = map(add, totalVidAvg, vidAvgDelays);
  i+=1;

#Opening the average files to write
navAvgFile = open("AvgFileNav.csv", 'wt');
vidAvgFile = open("AvgFileVid.csv", 'wt');

print "Output Written to AvgFileNav.csv and AvgFileVid.csv"

navWriter = csv.writer(navAvgFile);
navWriter.writerow( ('Range','AvgDelay','Abs(AvgDelay)') );
vidWriter = csv.writer(vidAvgFile);
vidWriter.writerow( ('Range','AvgDelay', 'Abs(AvgDelay)') );

idx = 0;
while idx < len(totalNavAvg):
  navWriter.writerow ( (str(200*idx)+'-'+str(200*(idx+1)), totalNavAvg[idx]/len(navFiles), abs(totalNavAvg[idx])/len(navFiles)  ));
  vidWriter.writerow ( (str(30*idx)+'-'+str(30*(idx+1)), totalVidAvg[idx]/len(vidFiles), abs(totalVidAvg[idx])/len(vidFiles) ));
  idx+=1;

#print len([name for name in os.listdir('.') if os.path.isfile(name)])
