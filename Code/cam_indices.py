import os
devs = os.listdir('/dev')
vid_indices = [int(dev[-1]) for dev in devs 
               if dev.startswith('video')]
vid_indices = sorted(vid_indices)
print(vid_indices)