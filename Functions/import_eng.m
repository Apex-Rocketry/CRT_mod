function [motor,I,description] = import_eng(filename,X)

f = fopen(filename,'r');

description = '';
descrip_flag = true;
table = [];
while not(feof(f))
    line = fgetl(f);
    if strcmp(line(1),';') && descrip_flag
        description = [description line(2:end)];
    elseif descrip_flag
        metadata = line;
        descrip_flag = false;
    else
        if isempty(table)
            table = [0 0];
            data = str2num(line);
        elseif all(table == 0)
            data2 = str2num(line);
            if not(data(1) == 0)
                dt = 0:data2(1)-data(1):data(1);
                int = interp1([0 data(1)],[0,data(2)],dt);
                table = [dt' int'; data2];
            else
                table = [data;data2];
            end
        else
            data = str2num(line);
            table = [table;data];
        end
    end
    
end
fclose(f);

if length(table) > 100
    dt = 0:table(end,1)/100:table(end,1);
    table = [dt' [interp1(table(:,1),table(:,2),dt)]'];
end


description = string(description);
idx = strfind(metadata,' ');
name = metadata(1:idx(1)-1);
diameter = str2double(metadata(idx(1)+1:idx(2)-1))/1000;
len = str2double(metadata(idx(2)+1:idx(3)-1))/1000;
fuel_mass = str2double(metadata(idx(4)+1:idx(5)-1));
engine_mass = str2double(metadata(idx(5)+1:idx(6)-1))-fuel_mass;
manufacturer = metadata(idx(6)+1:end);

motor_name = [manufacturer '_' name];

[M,I] = impulse_and_mass(table(:,1),table(:,2),fuel_mass);

thrustTable = [table M'+engine_mass];

motor = {'motor',motor_name,thrustTable,len,diameter,X};
disp(['Motor ' motor_name ' with total impulse of ' num2str(I) 'Ns was imported'])
end