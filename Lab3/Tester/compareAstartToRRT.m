
times = zeros(100,1);
totalPathsReal = zeros(100,1);
totalPathsConfig = zeros(100,1);
for i=1:50
    [time, totalPathReal, totalPathConfig] = calculatePathDistance(false, 'map4.txt');
    times = [times; time];
    totalPathsReal = [totalPathsReal; totalPathReal];
    totalPathsConfig = [totalPathsConfig; totalPathConfig];
end

avgTime = mean(times);
avgRealPathLength = mean(totalPathsReal);
avgConfigPathLength = mean(totalPathsConfig);

disp('Average Time: ');
disp(avgTime);
disp('Average Real Path Length: ');
disp(avgRealPathLength);
disp('Average Config Path Length: ');
disp(avgConfigPathLength);
