function generateTable(pathCoordinates, filename)
    % Load configuration
    config;

    % Process the path coordinates to extract y and z intercepts
    TV_Y = pathCoordinates(:, 1);
    TV_Z = pathCoordinates(:, 2);

    % Generate TV_X array with all elements set to 1
    TV_X = ones(size(TV_Y));

    % Calculate the y and z intercept ranges
    [y_min, y_max] = bounds(TV_Y);
    [z_min, z_max] = bounds(TV_Z);

    % Save the table to a file
    fileID = fopen(filename, 'w');
    fprintf(fileID, '#ifndef ARRAY_H\n');
    fprintf(fileID, '#define ARRAY_H\n\n');
    
    % Write TV_Y array
    fprintf(fileID, 'const double TV_Y[] = {\n');
    for i = 1:length(TV_Y)
        fprintf(fileID, '    %.6f', TV_Y(i));
        if i < length(TV_Y)
            fprintf(fileID, ',\n');
        else
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '};\n\n');
    
    % Write TV_Z array
    fprintf(fileID, 'const double TV_Z[] = {\n');
    for i = 1:length(TV_Z)
        fprintf(fileID, '    %.6f', TV_Z(i));
        if i < length(TV_Z)
            fprintf(fileID, ',\n');
        else
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '};\n\n');

    % Write TV_X array
    fprintf(fileID, 'const double TV_X[] = {\n');
    for i = 1:length(TV_X)
        fprintf(fileID, '    %.6f', TV_X(i));
        if i < length(TV_X)
            fprintf(fileID, ',\n');
        else
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '};\n\n');
    
    fprintf(fileID, '#endif // ARRAY_H\n');
    fclose(fileID);
end