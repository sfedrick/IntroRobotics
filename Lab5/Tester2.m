qCurr=[0,0,0,0,0,0];
qGoal=[1,0,0,0,0,0];
map=loadmap("map1.txt");
potentialFieldStep(qCurr, map.obstacles, qGoal)