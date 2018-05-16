#!/usr/bin/env python

from pocketsphinx import LiveSpeech

speech = LiveSpeech(
            verbose=False,
            sampling_rate=96000, # 48000
            buffer_size=4096, # 4096
            no_search=False,
            full_utt=False,
            hmm='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/en-us-Win10', # 'en-us-Win10', 'en-us-dist-packages'
            lm='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/7600.lm',
            dic='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/7600.dic'
        )

print 'listening...'
thres = 0.2
for phrase in speech:
    print '--------'
    print 'confidence', phrase.confidence()
    print 'hypothesis', phrase.hypothesis()
    