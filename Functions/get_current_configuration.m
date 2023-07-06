function conf = get_current_configuration()
    global configuration_name;
    text = fileread(fullfile("Configurations", configuration_name + ".json"));
    conf = jsondecode(text);
end
