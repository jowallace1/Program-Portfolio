fileName = "F15TC.xlsx";
F15TC = zeros(27,2);
readMatrix = readmatrix(fileName);
F15TC(2:27,1:2) = readMatrix(:,1:2);
plot(F15TC(:,1),F15TC(:,2))