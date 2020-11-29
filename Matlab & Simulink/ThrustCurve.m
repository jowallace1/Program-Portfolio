fileName = "E12TC.csv";
readMatrix = readmatrix(fileName);
TC = readMatrix(:,1:2);
plot(TC(:,1),TC(:,2))