%{
    This function takes as input a scan and applies an algorithm to detect the features of the scan
    itself.

    The algorithm is based on a seed generation that are then merged in order to create proper 
    segments. At the end they are processed in order to obtain the feature.

    The function returns:
        - seeds: the list of the recognized seed after the growing process;
        - features: a 2xN matrix containing the cartesian coordinates of the extracted features 
          after the processing of the seeds;
        - n_removed: number of removed features from the seeds due to their ineligibility; 
          this number can give an heuristic idea to the controller if the direction of the camera 
          is either good or bad.
%}
function [seeds, features, n_removed] = extract_features(scan)
    features = [];
    n_removed = 0;

    config = get_current_configuration();

    n_scan_points   = size(scan.polar_measures, 2);
    n_pt_min        = config.feature_extraction.n_point_generation;
    n_back          = config.feature_extraction.n_point_back_on_success;

    seeds = {};

    % --- Step 1: seed generation
    i = 1;
    while i < n_scan_points-n_pt_min
        if scan.polar_measures(1, i) ~= config.camera.d_max
            seed = Seed(scan, i, i+n_pt_min);
            
            if seed.is_valid_seed()
                seeds{end+1} = seed;
                i = i + n_pt_min - n_back - 1;
            end
        end

        i = i + 1;
    end


    % --- Step 2: seed merging
    can_continue = true;
    while can_continue
        starting_seed_size = length(seeds);
        tmp_seeds = seeds;

        i = 1;
        while i < size(tmp_seeds, 2)

            if tmp_seeds{i}.is_joinable_with(tmp_seeds{i+1})
                joint_seed = Seed(tmp_seeds{i}, tmp_seeds{i+1});

                if joint_seed.is_valid_seed()
                    tmp_seeds{i} = joint_seed;
                    tmp_seeds(i+1) = [];
                end
            end
            i = i + 1;
        end

        if starting_seed_size == size(tmp_seeds, 2)
            can_continue = false;
        end
        seeds = tmp_seeds;
    end

    
    % --- Step 3: seed growing
    for i = 1:length(seeds)
        seeds{i}.grow(scan);
    end


    % --- Step 3b: seed merging
    can_continue = true;
    while can_continue
        starting_seed_size = length(seeds);
        tmp_seeds = seeds;

        i = 1;
        while i < size(tmp_seeds, 2)

            if tmp_seeds{i}.is_joinable_with(tmp_seeds{i+1})
                joint_seed = Seed(tmp_seeds{i}, tmp_seeds{i+1});

                if joint_seed.is_valid_seed()
                    tmp_seeds{i} = joint_seed;
                    tmp_seeds(i+1) = [];
                end
            end
            i = i + 1;
        end

        if starting_seed_size == size(tmp_seeds, 2)
            can_continue = false;
        end
        seeds = tmp_seeds;
    end


    % --- Step 4: seed processing
    % Step 4a: remove unnecessary seeds
    i = 2;
    while i <= length(seeds)-1 % Processing of overlapping seeds
        prev = seeds{i-1};
        next = seeds{i+1};

        if prev.end_index >= next.start_index
            seeds(i) = [];
        else
            i = i + 1;
        end
    end

    i = 1;
    while i <= length(seeds) % Removing seeds not respecting the selection criterias
        
        seed_len    = seeds{i}.length();
        seed_n_pts  = seeds{i}.end_index - seeds{i}.start_index;

        cond1       = seed_len < config.feature_extraction.min_seed_length;
        cond2       = seed_n_pts < config.feature_extraction.min_seed_n_points;
        if cond1 || cond2 
            seeds(i) = [];
        else
            i = i+1;
        end
    end

    % Step 4b: extract the features
    if seeds{1}.start_index == 1 
        n_removed = n_removed + 1;
    else
        features(:, end+1) = seeds{1}.get_startpoint();
    end

    for i = 1:length(seeds)-1
        if seeds{i}.end_index > seeds{i+1}.start_index 
            features(:, end+1) = seeds{i}.compute_intersection(seeds{i+1});
        else
            features = [features, seeds{i}.get_endpoint(), seeds{i+1}.get_startpoint()];
        end
    end

    if seeds{end}.end_index == size(scan.cartesian_points, 2)
        n_removed = n_removed + 1;
    else 
        features(:, end+1) = seeds{end}.get_endpoint();
    end
end 
