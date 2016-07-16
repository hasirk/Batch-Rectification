./rectify -l leftimage_format -r rightimage_format -i intrinsic.yaml -e extrinsic.yaml -n num (default 1000) -cut cutheight (default 0)

# the program will automatically create folder under home/rectified~~

./rectify -l '/home/zh/experiments/Mar17201615:53/L0%04d.png' -r '/home/zh/experiments/Mar17201615:53/R0%04d.png'  -i '/home/zh/test_data/intrinsics.yml' -e '/home/zh/test_data/extrinsics.yml' -n 1620 -cut 324



./rectify -l '/home/zh/experiments/Mar17201615:59/L0%04d.png' -r '/home/zh/experiments/Mar17201615:59/R0%04d.png'  -i '/home/zh/test_data/intrinsics.yml' -e '/home/zh/test_data/extrinsics.yml' -n 1620 -cut 344







./stereo_movie '/home/zh/rectified/Mar19201610:57/L0%04d.png' '/home/zh/rectified/Mar19201610:57/R0%04d.png' 128





./stereo_movie '/home/zh/rectified/Mar19201611:12/L0%04d.png' '/home/zh/rectified/Mar19201611:12/R0%04d.png' 128