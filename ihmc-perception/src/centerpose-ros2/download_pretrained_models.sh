#!/bin/bash
# https://drive.google.com/drive/folders/16HbCnUlCaPcTg4opHP_wQNPsWouUlVZe

echo "Downloading models..."

mkdir -p models

# wget -nc means it will skip if already downloaded

# bike
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=11QLDbPzMiZYpsAWvwke0LPV2bYzAo1uk' -O models/bike_v1_140.pth
# book
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=16NTsoJZIeZpeVFTL_nxF5nhWISZEoEDo' -O models/book_v1_140.pth
# bottle
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1-vauJwkztE47__m90wyYx-5grXuy4ix4' -O models/bottle_v1_sym_12_140.pth
# camera
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1_CkUdhr_-Xr2RP5NihB2yLSlzS1_KLRT' -O models/camera_v1_140.pth
# cereal box
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1BhHnExf-fAI77XF6IGaIwnsK5r5OT1bF' -O models/cereal_box_v1_140.pth
# chair
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1PaNVJgRkyQjnNAVxOwQ9Ho3REAsmgtzf' -O models/chair_v1_140.pth
# cup
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1ClRdu_69-Dw-3MSO_iAs1ewzFbIr4rvK' -O models/cup_cup_v1_sym_12_140.pth
# mug
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=17ER2sw7rzTcpRNy3FsjKGRfI-gEX2I_R' -O models/cup_mug_v1_140.pth
# laptop
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1IFFuEevJnB4TdOJW-wgKADDlbZ_4bbpf' -O models/laptop_v1_140.pth
# shoe
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=16ybW3O1-86LzGxWzP6vS_0Dv6BNTA1p3' -O models/shoe_v1_140.pth
