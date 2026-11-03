On this project, I want to build a script to run and convert the raspberry pi zero reading to the mcap format.


The motivation on this issue is that, origanlly, the project of the speed trap was intended to record the raspberry pi video using mcap format, but, as the hardware is 
very weak (rpi zero v1.3), we discovered that the processor would not be able to install the ros2 because of the 32 bits processor. Instead of spending time testing the
ros2 capabilities on that hardware, we decided to record using simple things, such as the hardware video encoder to h264 to the raspberry pi and a simple csv for the radar.

The ingestion pipeline that we are building will be designed such as any mcap file recorded on the robots, and not only this project, to be ingested and processed to 
extract metadata and other information from those videos, so, this project, bust be a series of scrips that, given folders s0000N, that are available on a raspberry pi zero
of the radar project, we can convert them to the mcap to, later on, them to be catalog and stored properly using the other tools that we are building. 

Each recording of type s0000N, is intended to be split on the raspberry pi zero at each 5 minutes, one decision that we must is how to handle those multiple splits of the
same recorrding, if we joing every video and radar reading in the same mcap or, otherwise, if we use the 5 mins lens videos and each will be a mcap. I would prefer that each
recording (s00000N), to be joined and generate a single mcap. I use the button of the raspberry pi to close and launch new records when the context ask so, ideally, they
hold differnt enough, but, to turn each 5 mins in a single mcap is not bad also.

Regarding the sensors placement and other, i suggest that the ts of the each frame and radar reading on the video to be also recorded. If we "play"the mcap, we should be able
to generate the video and radar readings at the time that they were captured. We should also have an base description to hold the "tf "of the sensors for future use. It would
be important to investigate deeply the files to check which other metadata is available and would be important to have on the mcap. Also, for the radar reading, we should
discover which ros2 message format is recommded for the radar readings, so, a search of that purpose is recommended. Also, regarding camera parameters, the camera
that i use is not calibrated, but is a common raspberry pi v1.3 camera, the conversion scripts should allow to also input the camera info. I understand that those 
"fixed" parameters, such as the camera info and others to be available through config files. 

The project should be, ideally, written in python, and use uv for dependencies managements. It should be well written, organized, have a readme (use the readme skill). 
and open source community worthy. If necessary, inspect the other packages within our workspace (../), to check the patterns for file recording or how we structured
the readme for this project. The script should be simple to use, and yeild a professional file that would be ingested into our catalog. The catalog and ingestion itself
are out of scope of this project. 

USe the brainstorm skill and analyze thoroughly this propposal. Discover weak points and other definitions that are necessary for us to have a reliable and professional tool
to add to our projects.
