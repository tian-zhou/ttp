#!/usr/bin/env python

from pocketsphinx import LiveSpeech

speech = LiveSpeech(
    verbose=False,
    sampling_rate=48000,
    buffer_size=2048,
    no_search=False,
    full_utt=False,
    hmm='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/en-us-Win10', # 'en-us-Win10', 'en-us-dist-packages'
    lm='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/9711.lm',
    dic='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/9711.dic'
)

print 'listening...'
thres = 0.2
for phrase in speech:
    print '--------'
    print 'confidence', phrase.confidence()
    print 'hypothesis', phrase.hypothesis()
    