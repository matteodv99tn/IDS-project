
function A = rototranslation_matrix(x, y, theta)
    A = translation_matrix(x, y) * rotation_matrix(theta);
end
