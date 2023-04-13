function seeds = extract_features(scan)
    config = get_current_configuration();

    n_scan_points   = size(scan.polar_measures, 2);
    n_pt_min        = config.feature_extraction.n_point_generation;
    n_back          = config.feature_extraction.n_point_back_on_success;

    seeds = {};

    % Seed generation
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


    % Seed joining 
    can_continue = true;
    while can_continue
        fprintf("----\n");
        starting_seed_size = size(seeds, 2);
        fprintf("Starting size: %d\n", starting_seed_size);
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

        fprintf("After processing: %d\n", size(tmp_seeds, 2));

        if starting_seed_size == size(tmp_seeds, 2)
            fprintf("Should terminate\n");
            can_continue = false;
        end
        seeds = tmp_seeds;
    end


      

end 
