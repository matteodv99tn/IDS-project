

data = cell(3,1);

data{1} = rand(5,1);
data{2} = rand(9,1);
data{3} = rand(1,1);

mean_vect = zeros(3,1);
for i = 1:3 
    mean_vect(i) = mean(data{i});
end

mean_2 = cellfun(@mean, data, "UniformOutput", false);
