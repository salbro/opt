module Utils

export flowcost_top_caldata, flowcost_whole_caldata, flowcost_barry, hasbbdx_caldata, hasbbdx_barry, reverse_dict, caldata_demands, caldata_whole_demands, barry_demands, get_hubs_to_cycles, caldata_cycles, caldata_whole_cycles, barry_cycles, caldata_type_0_nodes, whole_caldata_type_0_nodes, barry_type_0_nodes, get_arc_0_key, get_arc_1_key, get_arc_2_key, get_arc_3_key, get_node_key, adm_cost_per_unit_data, bbdx_cost_per_unit_data, bidirectional_arc_capacity_caldata, bidirectional_arc_capacity_barry,  adm_capacity_caldata, adm_capacity_barry, adm_cost, get_node_and_arc_dicts

using JuMP, Gurobi
using Combinatorics


adm_cost_per_unit_data = 2;
bbdx_cost_per_unit_data = 4; # plus 2 for each ADM
bidirectional_arc_capacity_caldata = 24000; 
bidirectional_arc_capacity_barry = 48000;
adm_capacity_caldata = 48000; # because each ADM is attached to two arcs
adm_capacity_barry = 96000; # because each ADM is attached to two arcs
adm_cost = 1000000;

############################################# DATA #############################################
caldata_whole_cycles = Dict("ABEC"=>4, "BDE"=>2, "EGLKIHF"=>3, "HIKJ"=>2, "KLNM"=>3);
caldata_cycles = Dict("ABEC"=>4, "BDE"=>2);

caldata_type_0_nodes = ["A", "B", "C", "D", "E"]
whole_caldata_type_0_nodes = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N"];
barry_type_0_nodes = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U"];

caldata_demand_dict = Dict("A" => Dict("B" => 38000, "D"=>7000, "E"=> 5000),
    "B" => Dict( "D"=>19000, "C"=> 1000, "A"=>42000, "E"=>7000),
    "D" => Dict("C"=>1000, "A"=>12000),
    "C" => Dict("A"=>1000)
);

caldata_demands = Dict()
for hub in keys(caldata_demand_dict)
    for other_hub in keys(caldata_demand_dict[hub])
        caldata_demands[hub*other_hub] = caldata_demand_dict[hub][other_hub]
    end
end

caldata_whole_demand_dict = Dict(
    "A" => Dict("B" => 38000, "D"=>7000, "E"=> 5000, "N"=>27000, "J"=>1000),
    "B" => Dict("K"=>13000, "D"=>19000, "C"=> 1000, "A"=>42000, "N"=>31000, "E"=>7000, "F"=>6000,
                "J"=>9000, "H"=>2000),
    "D" => Dict("C"=>1000, "A"=>12000, "N"=>1000, "J"=>2000),
    "C" => Dict("A"=>1000),
    "K"=>Dict("M"=>2000, "A"=>11000, "N"=>32000, "J"=>14000, "H"=>6000, "L"=>6000),
    "M"=>Dict("J"=>1000),
    "N"=>Dict("J"=>20000, "L"=>11000),
    "E"=>Dict("G"=>3000),
    "F"=>Dict("J"=>2000),
    "J"=>Dict("H"=>4000, "L"=>2000)
);

caldata_whole_demands = Dict()
for hub in keys(caldata_whole_demand_dict)
    for other_hub in keys(caldata_whole_demand_dict[hub])
        caldata_whole_demands[hub*other_hub] = caldata_whole_demand_dict[hub][other_hub]
    end
end


### BARRY

barry_cycles = Dict("ABLNIKMQHD"=>4, "IPTUJFK"=>3, "MEGRCOSQ"=>6)

barry_demands = Dict(
    "AB"=>66000, "AC"=>1000, "AD"=>34000, "AE"=>2000, "AF"=>2000, "AG"=>23000, "AH"=>10000, "AI"=>16000, "AJ"=>13000,
    "AK"=>6000, "AL"=>9000, "AM"=>23000, "AN"=>17000, "AO"=>57000, "AP"=>9000, "AQ"=>12000, "AR"=>1000, "AS"=>21000,
    "TF"=>2000, "TI"=>3000, "TJ"=>1000, "TP"=>30000, 
    "BF"=>1000, "BG"=>3000, "BI"=>8000, "BJ"=>3000, "BK"=>2000, "BL"=>2000, "BN"=>1000, "BO"=>1000, "BP"=>2000, "BS"=>1000,
    "CG"=>14000, "CI"=>1000, "CK"=>1000, "CO"=>21000, "CR"=>3000, "CS"=>3000,
    "DH"=>2000,
    "EG"=>32000, "EI"=>8000, "EJ"=>8000, "EK"=>8000, "EM"=>2000, "ER"=>1000, 
    "FG"=>6000, "FI"=>19000, "FJ"=>61000, "FK"=>10000, "FP"=>2000, "FS"=>1000,
    "GI"=>13000, "GJ"=>23000, "GK"=>4000, "GM"=>17000, "GO"=>118000, "GP"=>6000, "GR"=>177000, "GS"=>15000,
    "HM"=>1000,
    "IJ"=>32000, "IK"=>20000, "IL"=>1000, "IM"=>10000, "IN"=>31000, "IO"=>18000, "IP"=>31000, "IR"=>1000,
    "JK"=>24000, "JM"=>8000, "JN"=>3000, "JO"=>16000, "JP"=>11000, "JU"=>9000, "JR"=>1000,
    "KM"=>2000, "KN"=>2000, "KO"=>10000, "KP"=>2000, "KR"=>2000,
    "LN"=>10000, 
    "MO"=>4000, "MP"=>1000, "MS"=>2000,
    "NO"=>1000,
    "OP"=>9000, "OR"=>23000, "OS"=>113000,
    "RS"=>4000
    )

############################################# END DATA #############################################

function flowcost_whole_caldata(hub1hub2)
    costs = Dict("KM"=>1,"MN"=>1, "NL"=>1, "LK"=>1, "JH"=>1, "HI"=>1, "IK"=>1, "KJ"=>1,
                "LG"=>2, "GE"=>3, "EF"=>3, "FH"=>2, "BD"=>1, "DE"=>1, "EB"=>1, "BA"=>1,
                "AC"=>1, "CE"=>1)
    
    if hub1hub2 in keys(costs)
        return costs[hub1hub2]
    elseif reverse(hub1hub2) in keys(costs)
        return costs[reverse(hub1hub2)]
    else
        return 0
    end
end

function flowcost_top_caldata(hub1hub2)
    costs = Dict("BD"=>1, "DE"=>1, "EB"=>1, "BA"=>1, "AC"=>1, "CE"=>1)
    
    if hub1hub2 in keys(costs)
        return costs[hub1hub2]
    elseif reverse(hub1hub2) in keys(costs)
        return costs[reverse(hub1hub2)]
    else
        return 0
    end
end

function flowcost_barry(hub1hub2)
    costs = Dict(
    "AB"=>1.08, "BL"=>1.08, "LN"=>1.04, "NI"=>1.07, "IK"=>1.05, "KM"=>1.07, "MQ"=>1.06, "QH"=>1.01, "HD"=>1.03, 
        "DA"=>1.03, "GR"=>1.01, "RC"=>1.05, "CO"=>1.04, "OS"=>1.04, "SQ"=>1.10, "ME"=>1.06, "EG"=>1.05, "JF"=>1.01,
        "FK"=>1.03, "IP"=>1.04, "PT"=>1.03, "TU"=>1.02, "UJ"=>1.02
    )
    
    if hub1hub2 in keys(costs)
        return costs[hub1hub2]
    elseif reverse(hub1hub2) in keys(costs)
        return costs[reverse(hub1hub2)]
    else
        return 0
    end
end

function hasbbdx_caldata(hub)
    return ! (hub == "C" || hub=="F" || hub =="G" || hub=="I" || hub=="M")
end;

function hasbbdx_barry(hub)
    return ! (hub == "D" || hub=="H" || hub =="L" || hub=="N" || hub=="S" || hub=="C" || hub=="R" || hub=="E"
             || hub=="F" || hub=="U" || hub=="T")
end;


function reverse_dict(dict)
    rev = Dict()
    for k in keys(dict)
       rev[dict[k]] = k 
    end
    return rev
end

function get_arc_0_key(hub, cycle, ring)
    "hub_" * hub * "_cycle_" * cycle * "_ring_" * string(ring) * "_to_hub_" * hub
end

function get_arc_1_key(hub, cycle, ring)
    "hub_" * hub * "_to_hub_" * hub * "_cycle_" * cycle * "_ring_" * string(ring)
end

function get_arc_2_key(orig_hub, dest_hub, cycle, ring)
    key = "hub_" * string(orig_hub) * "_cycle_" * cycle * "_ring_" * 
                    string(ring) * "_to_hub_" * dest_hub * "_cycle_" *
                    cycle * "_ring_" * string(ring)
    return key
end

function get_arc_3_key(hub, cycle_orig, cycle_dest, ring_orig, ring_dest)
    return "hub_"*hub*"_cycle_"*cycle_orig*"_ring_"*string(ring_orig) *
            "_to_hub_"*hub*"_cycle_"*cycle_dest*"_ring_"*string(ring_dest)
end

function get_node_key(hub, cycle, ring)
    "hub_" * hub * "_cycle_" * cycle * "_ring_" * string(ring)
end
    

function get_hubs_to_cycles(cycle_dict)
    hubs_to_cycles = Dict()
    for cycle in keys(cycle_dict)
        for hub in cycle
            if string(hub) in keys(hubs_to_cycles)
                push!(hubs_to_cycles[string(hub)], cycle)
            else
                hubs_to_cycles[string(hub)] = [cycle]
            end
        end
    end
    
    return hubs_to_cycles
end

function get_node_and_arc_dicts(cycles, hasbbdx, type_0_nodes)
    ##### NODES ######
    node_indices_to_descriptions = Dict()
    index = 1

    for cycle in keys(cycles)
        for hub in cycle
            for ring in 1:cycles[cycle]
                node_indices_to_descriptions[index] = get_node_key(hub, cycle, ring)
                index += 1
            end
        end
    end
    
    node_descriptions_to_indices = reverse_dict(node_indices_to_descriptions)
    num_nodes = length(node_indices_to_descriptions)

    hubs_to_cycles = get_hubs_to_cycles(cycles)
    
    ##### ARCS ######
    arc_indices_to_descriptions = Dict()
    
    ind = 1
    # type 0 and type 1 arcs: from each hub to ring, and from each ring to the hub, respectively
    for hub in type_0_nodes
        for cycle in hubs_to_cycles[hub]
            for ring in 1:cycles[cycle]
                # from city to ring
                arc_indices_to_descriptions[ind] = get_arc_1_key(hub, cycle, ring)
                ind += 1
                # from ring to city
                arc_indices_to_descriptions[ind] = get_arc_0_key(hub, cycle, ring)
                ind += 1
            end
        end
    end

#     println("after type 0 and 1: "*string(ind))
    
    # type 2 arcs: one hub to the next along a ring (bidirectional, not symmetric)
    for cycle in keys(cycles)
        for ring in 1:cycles[cycle]
            for i in 1:(length(cycle)-1)                
                arc_indices_to_descriptions[ind] = get_arc_2_key(cycle[i], cycle[i+1], cycle, ring)
                ind += 1
                arc_indices_to_descriptions[ind] = get_arc_2_key(cycle[i+1], cycle[i], cycle, ring)
                ind += 1
            end
            arc_indices_to_descriptions[ind] = get_arc_2_key(cycle[1], cycle[end], cycle, ring)
            ind += 1
            arc_indices_to_descriptions[ind] = get_arc_2_key(cycle[end], cycle[1], cycle, ring)
            ind += 1
        end
    end
      
#     println("after type 2: "*string(ind))

    # type 3 arcs: These go from one ring to another at a hub that has a BBDX. 
    # Any commodity may flow on these arcs, but only if an ADM is installed at the hub on both rings. 
    for hub in type_0_nodes
#         if !hasbbdx(hub)
#             continue ----- its ok to have the variable but they will be set to zero in the constraints
#         end
        cycles_at_hub = hubs_to_cycles[hub]
        for cycle_num in 1:length(cycles_at_hub)
            # in-cycle transfers: across rings
            for pair in collect(combinations(collect(1:cycles[cycles_at_hub[cycle_num]]), 2))
                ring1, ring2 = pair
                arc_indices_to_descriptions[ind] = get_arc_3_key(hub, cycles_at_hub[cycle_num], cycles_at_hub[cycle_num], ring1, ring2)
                ind += 1
                arc_indices_to_descriptions[ind] = get_arc_3_key(hub, cycles_at_hub[cycle_num], cycles_at_hub[cycle_num], ring2, ring1)
                ind += 1
            end

            # out-of-cycle transfers
            for other_cycle_num in append!(collect(1:cycle_num-1), collect(cycle_num+1:length(cycles_at_hub)))
                # for each ring in THIS cycle, make a link to the other ring in other cycles
                for ring in 1:cycles[cycles_at_hub[cycle_num]]
                    for other_ring in 1:cycles[cycles_at_hub[other_cycle_num]]
                        if cycle_num == other_cycle_num
                            print(cycle_num)
                        end

                        arc_indices_to_descriptions[ind] = get_arc_3_key(hub, cycles_at_hub[cycle_num], cycles_at_hub[other_cycle_num], ring, other_ring)
                        ind += 1
                    end
                end
            end
        end
    end
    
    arc_descriptions_to_indices = reverse_dict(arc_indices_to_descriptions)
    
    return (node_descriptions_to_indices, node_indices_to_descriptions, 
            arc_descriptions_to_indices, arc_indices_to_descriptions)
end


end