#!/usr/bin/env python
# coding=utf-8

from lejulib import *


def main():
    node_initial()

    try:
        u4u6bb5u821eu8e48u548u5e76 = [
            ([0,1,1,-1,-1,-1,0,1,-1,1,1,1,0,-70,-22,0,70,22,0,0,0,-5],520,0),
            ([15,-8,-3,-5,-3,-8,-15,8,3,5,3,8,-8,-50,-30,8,50,30,0,0,-17,21],332,0),
            ([0,-5,0,0,0,-5,0,5,0,0,0,5,-8,-70,-15,8,70,15,0,0,0,21],520,0),
            ([-10,-8,-3,-5,-3,-8,10,8,3,5,3,8,-9,-50,-30,6,50,30,0,0,0,0],332,0),
            ([-5,-8,3,-5,-3,-8,5,8,-3,5,3,8,-9,-70,-15,6,70,15,0,0,0,0],520,0),
            ([20,-12,-3,-5,-3,-12,-15,12,3,5,3,12,-9,-50,-30,6,50,30,0,0,29,18],332,0),
            ([15,-12,0,0,0,-12,-10,12,0,0,0,12,-9,-70,-15,6,70,15,0,0,29,18],488,0),
            ([-3,-14,1,-1,-1,-14,3,14,-1,1,1,14,-9,-50,-30,6,50,30,0,0,0,0],332,0),
            ([-3,-10,0,0,0,-10,3,10,0,0,0,10,-9,-70,-15,6,70,15,0,0,0,0],520,0),
            ([-3,-10,0,0,0,-10,3,10,0,0,0,10,-9,-70,-15,6,70,15,0,0,0,0],988,0),
            ([16,-12,-3,-1,-1,-14,-14,14,3,1,1,14,-9,-50,-30,6,50,30,0,0,29,18],312,0),
            ([10,-12,0,0,0,-12,-10,12,0,0,0,12,-9,-70,-15,6,70,15,0,0,29,18],520,0),
            ([-3,-10,-3,-5,-3,-10,3,10,3,5,3,10,-9,-50,-30,6,50,30,0,0,0,0],312,0),
            ([-3,-8,0,0,0,-8,3,8,0,0,0,8,-9,-70,-15,6,70,15,0,0,0,0],520,0),
            ([20,-8,-3,-1,-1,-8,-15,8,3,1,1,8,-9,-50,-30,6,50,30,0,0,-36,25],312,0),
            ([15,-8,-3,-1,-1,-8,-10,8,3,1,1,8,-9,-70,-15,6,70,15,0,0,0,20],520,0),
            ([-3,-5,-3,-5,-3,-5,3,5,3,5,3,5,-9,-50,-30,6,50,30,0,0,0,0],312,0),
            ([-3,-5,1,-1,-1,-5,3,5,-1,1,1,5,-9,-70,-15,6,70,15,0,0,0,0],520,0),
            ([0,-7,1,-1,-1,-7,0,7,-1,1,1,7,-9,-50,-30,6,50,30,0,0,0,0],520,0),
            ([0,-5,1,-1,-1,-10,0,5,-1,1,1,7,36,-12,-54,58,8,49,0,0,0,0],936,0),
            ([-40,-10,20,-15,0,-8,-40,10,5,0,5,10,-47,-85,-37,23,33,26,0,0,0,0],1560,0),
            ([-40,-8,12,-20,0,-8,-40,10,15,0,7,10,-107,-66,-16,122,79,26,0,0,0,0],1333,0),
            ([-40,-10,10,-20,0,-8,-40,7,15,0,5,7,-37,-75,-45,50,73,61,0,0,-9,25],1333,0),
            ([40,-10,5,0,5,-10,40,10,-20,15,0,8,-30,-68,-51,54,84,43,0,0,0,0],1560,0),
            ([-3,-8,0,0,0,-8,-3,8,0,0,0,8,-68,-75,-54,6,72,11,0,0,0,0],1300,0),
            ([3,-12,65,-70,-20,-12,-3,12,-65,70,20,12,-92,-82,-74,54,79,54,0,0,0,23],830,0),
            ([0,-15,65,-71,-23,-16,0,13,-69,80,21,12,-123,-10,-24,54,79,54,0,0,-58,23],832,0),
            ([0,-15,65,-71,-23,-16,0,13,-69,80,21,12,-155,-70,-47,54,79,54,0,0,-58,-3],624,0),
            ([0,-15,65,-71,-23,-16,0,13,-69,80,21,12,-123,-5,-28,54,79,54,0,0,-58,23],778,0),
            ([0,-15,65,-71,-23,-16,0,13,-69,80,21,12,-155,-70,-47,54,79,54,0,0,-58,-3],728,0),
            ([0,-5,44,-50,-13,-16,0,13,-47,63,21,12,-41,-44,-12,122,77,57,0,0,-1,1],832,0),
            ([0,-8,0,0,0,-8,0,8,0,0,0,6,-122,-56,-37,0,70,5,0,0,-9,-15],832,0),
            ([0,-8,16,-34,-18,-8,0,8,-16,34,18,8,-76,-82,-69,46,73,68,0,0,29,8],750,0),
            ([-5,-5,0,0,0,-5,-5,5,0,0,0,5,-76,-82,-69,130,85,61,0,0,0,-2],707,0),
            ([0,-5,0,-5,-4,-5,0,5,0,5,4,5,-103,-73,-69,86,85,64,0,0,0,27],707,0),
            ([-5,-5,0,0,0,-5,-5,5,0,0,0,5,-76,-82,-69,130,85,61,0,0,0,-2],707,0),
            ([0,-8,39,-34,-4,-8,0,8,-39,34,4,6,-14,-21,-38,61,75,70,0,0,0,27],572,0),
            ([3,-14,45,-5,15,-10,-3,2,-45,5,-15,6,-53,-80,-56,48,37,19,0,0,20,-10],832,0),
            ([3,-8,43,-5,15,-8,-3,8,-43,5,-15,8,-56,-62,-35,56,59,31,0,0,0,10],624,0),
            ([3,-2,42,-5,15,-6,-3,14,-42,5,-15,10,-60,-45,-15,65,81,44,0,0,-20,-10],624,0),
            ([3,-8,42,-12,10,-8,-3,8,-42,12,-10,8,-56,-62,-35,56,59,31,0,0,0,10],624,0),
            ([3,-14,43,-20,5,-10,-3,2,-43,20,-5,6,-53,-80,-56,48,37,19,0,0,20,-10],624,0),
            ([0,-13,3,-20,-6,-10,0,11,3,20,11,13,-88,-68,-74,16,65,26,0,0,-33,24],936,0),
            ([0,-13,15,-20,-6,-10,0,11,-8,20,11,13,-61,-51,-50,16,65,26,0,0,-30,3],936,0),
            ([0,-13,3,-20,-6,-10,0,11,3,20,11,13,-88,-68,-74,16,65,26,0,0,-33,24],936,0),
            ([0,0,0,0,0,-8,0,8,0,0,0,6,-153,-38,-48,157,23,68,0,0,0,0],1019,0),
            ([0,-17,0,0,-1,-17,0,-10,0,0,0,-11,-30,-73,-10,0,50,30,0,0,-53,25],1352,0),
            ([0,-15,67,-82,-16,-11,-58,-12,0,0,6,-12,0,-50,-30,126,66,62,0,0,35,9],1976,0),
            ([0,-15,67,-82,-16,-11,-58,-12,0,0,6,-12,0,-50,-30,126,66,62,0,0,35,9],832,0),
            ([0,-9,0,-29,-28,-11,0,-5,0,0,0,-12,-37,-12,-24,0,8,30,0,0,0,0],1248,0),
            ([0,-35,29,-50,-19,-21,0,-14,-21,32,11,-7,-63,-68,-74,16,65,26,0,0,-7,28],936,0),
            ([0,-9,45,-61,-19,-11,-39,3,-38,52,8,6,-22,-9,-17,66,68,66,0,0,-58,4],839,0),
            ([0,-34,30,-22,4,-23,0,-12,-23,13,-8,0,-126,-42,-12,126,73,61,0,0,-48,-19],847,0),
            ([38,-13,30,-22,0,-14,46,11,-23,13,-5,16,-68,-48,-16,118,51,61,0,0,-49,12],783,0),
            ([0,-34,30,-22,4,-25,0,-12,-23,13,-8,0,-126,-42,-12,126,73,61,0,0,-48,-19],783,0),
            ([0,-6,16,-34,-18,-8,0,10,-16,34,18,11,-98,-83,-10,0,28,85,0,0,0,0],751,0),
            ([0,-31,40,-44,-13,-28,0,-6,-62,73,21,-12,-98,-83,-64,0,-3,20,0,0,54,0],733,0),
            ([0,-31,40,-42,-5,-28,0,-9,-56,73,21,-12,-98,-67,-55,94,61,86,0,0,0,0],748,0),
            ([0,-31,40,-42,-5,-28,0,-9,-55,73,21,-12,5,-19,-13,6,12,9,0,0,57,0],852,0),
            ([0,-31,40,-42,-5,-28,0,-9,-55,73,21,-12,-98,-83,-64,94,66,72,0,0,5,0],780,0),
            ([0,-6,16,-34,-18,-8,0,10,-16,34,18,11,-82,-66,-38,62,28,20,0,0,-28,13],832,0),
            ([0,-6,16,-34,-18,-8,0,10,-16,34,18,11,44,-54,-13,-34,37,14,0,0,0,-12],1040,0),
            ([-40,-10,27,-20,6,-4,-42,3,-10,8,-2,9,-47,-85,-37,107,33,26,0,0,0,0],1248,0),
            ([40,-10,51,-65,-22,-9,40,10,-20,13,-5,17,-88,-19,-66,14,42,9,0,0,-6,10],1040,0),
            ([40,-10,-11,0,0,-9,40,10,3,13,7,8,32,-61,-10,62,30,14,0,0,0,0],1040,0),
            ([40,-10,0,0,0,-9,40,10,-20,13,-5,8,-30,-61,-10,82,73,14,0,0,0,0],832,0),
            ([40,-10,-11,0,0,-9,40,10,3,13,7,8,32,-17,-10,-18,1,14,0,0,0,0],988,0),
            ([0,-6,16,-34,-18,-8,0,10,-16,34,18,11,-130,-28,-70,14,23,85,0,0,0,0],1242,0),
            ([0,-34,13,-43,-16,-28,0,-12,-22,55,20,-5,-126,-42,-12,126,73,61,0,0,-48,-19],1242,0),
            ([0,-34,13,-43,-16,-28,0,-12,-22,55,20,-5,-126,-42,-12,126,73,61,0,0,-48,-19],732,0),
            ([0,-34,13,-43,-16,-28,0,-12,-22,55,20,-5,-126,-42,-12,126,73,61,0,0,-48,-19],50,0),
            ([0,-10,54,-63,-18,-8,0,6,-50,60,22,6,-18,-21,-10,10,1,16,0,0,0,22],832,0),
            ([0,-10,54,-63,-18,-8,0,6,-50,60,22,6,-30,-75,-10,10,1,16,0,0,0,22],520,0),
            ([0,-10,54,-63,-18,-8,0,6,-50,60,22,6,-126,-75,-10,10,1,16,0,0,0,-4],2080,0),
            ([0,-10,54,-63,-18,-8,0,6,-50,60,22,6,-112,-75,-10,10,1,16,0,0,0,-4],520,0),
            ([0,-10,54,-63,-18,-8,0,6,-50,60,22,6,-112,-75,-10,46,75,10,0,0,0,23],728,0),
            ([0,-8,16,-34,-18,-8,0,8,-16,34,18,6,-112,-75,-10,118,75,10,0,0,0,-4],2080,0),
            ([0,-8,16,-34,-18,-8,0,8,-16,34,18,6,-103,-75,-10,108,75,10,0,0,0,-4],312,0),
            ([0,-8,16,-34,-18,-8,0,8,-16,34,18,6,-112,-75,-10,118,75,10,0,0,0,-4],312,0),
            ([33,-8,16,-34,-18,-8,40,8,-16,34,18,6,-45,-72,-62,54,81,54,0,0,0,26],1248,0),
            ([0,-8,0,0,0,-8,0,4,0,0,0,6,-34,-68,-13,38,66,11,0,0,0,3],1040,0),
            ([0,-8,0,0,0,-8,0,4,0,0,0,6,-34,-68,-13,38,66,11,0,0,0,3],520,0),
            ([0,-13,16,-34,-18,-10,0,11,-16,34,18,7,0,-50,-30,0,50,30,0,0,0,18],832,0),
            ([0,-13,16,-34,-18,-10,0,11,-16,34,18,7,0,-50,-30,0,50,30,0,0,0,18],1664,0),
            ([49,-4,32,-41,-18,-10,0,11,-33,43,18,7,-108,-72,-70,7,63,30,0,0,-27,25],728,0)
        ]
        music = [("say_yeah.mp3", 0)] 
        client_action.music_action(music, u4u6bb5u821eu8e48u548u5e76)


    except Exception as err:
        serror(err)
    finally:
        finishsend()

if __name__ == '__main__':
    main()