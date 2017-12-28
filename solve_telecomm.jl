using JuMP, Gurobi
using Combinatorics
include("Utils.jl");
using Utils
caldata_top, caldata_whole, barry = 0,1,2

model = barry


## caldata top ##
if model==caldata_top
    demands = caldata_demands
    cycles = caldata_cycles
    getflowcost = flowcost_top_caldata
    hasbbdx = hasbbdx_caldata
    arc_capacity = bidirectional_arc_capacity_caldata
    type_0_nodes = caldata_type_0_nodes

elseif model==caldata_whole
    demands = caldata_whole_demands
    cycles = caldata_whole_cycles
    getflowcost = flowcost_whole_caldata
    hasbbdx = hasbbdx_caldata
    arc_capacity = bidirectional_arc_capacity_caldata
    type_0_nodes = whole_caldata_type_0_nodes

elseif model==barry
    demands = barry_demands
    cycles = barry_cycles
    getflowcost = flowcost_barry
    hasbbdx = hasbbdx_barry
    arc_capacity = bidirectional_arc_capacity_barry
    type_0_nodes = barry_type_0_nodes
end


commodity_keys = collect(keys(demands))
demand_vector = zeros(1:length(demands))
# commodities = 1:length(demands)
commodities = Dict()
for (i, key) in enumerate(commodity_keys)
    commodities[i] = key
    demand_vector[i] = demands[key]
end

hubs_to_cycles = get_hubs_to_cycles(cycles)
n_commodities = length(commodities)

node_descriptions_to_indices, node_indices_to_descriptions, 
    arc_descriptions_to_indices, arc_indices_to_descriptions =
            get_node_and_arc_dicts(cycles, hasbbdx, type_0_nodes)

num_arcs = length(arc_indices_to_descriptions)
n_type_1_nodes = length(node_indices_to_descriptions)

m = Model(solver=GurobiSolver())
BIGM = arc_capacity + 1 # the max arc capacity + 1

# decision variables
@variable(m, arcflows[1:num_arcs, 1:n_commodities] >= 0)
@variable(m, adms[1:n_type_1_nodes], Bin)

# constraints
# for each cycle
for cycle in keys(cycles)
    for ring in 1:cycles[cycle]
        # hit every arc in the cycle
        for i in 1:(length(cycle)-1)            
            # The total bi-directional flow on an arc can be no more than the bi-directional capacity.
            direction_1_key = get_arc_2_key(cycle[i], cycle[i+1], cycle, ring)
            direction_2_key = get_arc_2_key(cycle[i+1], cycle[i], cycle, ring)

            @constraint(m, [i=1:length(arcflows)], 
                sum(arcflows[arc_descriptions_to_indices[direction_1_key],j]
                + arcflows[arc_descriptions_to_indices[direction_2_key],j] for j in 1:n_commodities)
                <= arc_capacity)
        end

        direction_1_key = get_arc_2_key(cycle[1], cycle[end], cycle, ring)
        direction_2_key = get_arc_2_key(cycle[end], cycle[1], cycle, ring)

        @constraint(m, [i=1:length(arcflows)], 
            sum(arcflows[arc_descriptions_to_indices[direction_1_key], j]
            + arcflows[arc_descriptions_to_indices[direction_2_key], j] for j in 1:n_commodities)
            <= arc_capacity)
    end
end

for hub in type_0_nodes
    for cycle in hubs_to_cycles[hub]
        for ring in 1:cycles[cycle]
            for j in 1:n_commodities
                type_0_arc_key = get_arc_0_key(hub, cycle, ring)
                type_1_arc_key = get_arc_1_key(hub, cycle, ring)
                node_key = get_node_key(hub, cycle, ring)

                # No commodities may flow on a Type 0 arc unless their destination is at that hub.
                if ! endswith(commodities[j], hub)
                    @constraint(m, arcflows[arc_descriptions_to_indices[type_0_arc_key],j] == 0)
                end

                # • No commodities may flow on a Type 1 arc unless their origin is at that hub.
                if ! startswith(commodities[j], hub)
                    @constraint(m, arcflows[arc_descriptions_to_indices[type_1_arc_key],j] == 0)
                end

                # No commodities may flow on a Type 0 (city to ring) arc unless an ADM is installed on that ring at that hub.
                @constraint(m, arcflows[arc_descriptions_to_indices[type_0_arc_key],j]
                    <= adms[node_descriptions_to_indices[node_key]]*BIGM)

                # No commodities may flow on a Type 1 (ring to city) arc unless an ADM is installed on that ring at that hub.
                @constraint(m, arcflows[arc_descriptions_to_indices[type_1_arc_key],j]
                    <= adms[node_descriptions_to_indices[node_key]]*BIGM)

            end
        end
    end
end

# No commodities may flow on a Type 3 arc unless an ADM is installed at that hub on both rings.
for hub in type_0_nodes    
    cycles_at_hub = hubs_to_cycles[hub]
    for cycle_num in 1:length(cycles_at_hub)
        # in-cycle transfers: across rings
        for pair in collect(combinations(collect(1:cycles[cycles_at_hub[cycle_num]]), 2))
            ring1, ring2 = pair
            arc_key_dir1 = get_arc_3_key(hub, cycles_at_hub[cycle_num], cycles_at_hub[cycle_num], ring1, ring2)
            arc_key_dir2 = get_arc_3_key(hub, cycles_at_hub[cycle_num], cycles_at_hub[cycle_num], ring2, ring1)

            node_key_one = get_node_key(hub, cycles_at_hub[cycle_num], ring1)
            node_key_two = get_node_key(hub, cycles_at_hub[cycle_num], ring2)

            for j in 1:n_commodities
                if ! hasbbdx(hub)
                    @constraint(m, arcflows[arc_descriptions_to_indices[arc_key_dir1],j] == 0)
                    @constraint(m, arcflows[arc_descriptions_to_indices[arc_key_dir2],j] == 0)
                else
                    @constraint(m, arcflows[arc_descriptions_to_indices[arc_key_dir1],j]
                            <= adms[node_descriptions_to_indices[node_key_one]]*BIGM)
                    @constraint(m, arcflows[arc_descriptions_to_indices[arc_key_dir1],j]
                            <= adms[node_descriptions_to_indices[node_key_two]]*BIGM)

                    @constraint(m, arcflows[arc_descriptions_to_indices[arc_key_dir2],j]
                            <= adms[node_descriptions_to_indices[node_key_one]]*BIGM)
                    @constraint(m, arcflows[arc_descriptions_to_indices[arc_key_dir2],j]
                            <= adms[node_descriptions_to_indices[node_key_two]]*BIGM)
                end

            end

        end

        # out-of-cycle transfers
        for other_cycle_num in append!(collect(1:cycle_num-1), collect(cycle_num+1:length(cycles_at_hub)))
            # for each ring in THIS cycle, make a link to the other ring in other cycles
            for ring in 1:cycles[cycles_at_hub[cycle_num]]
                for other_ring in 1:cycles[cycles_at_hub[other_cycle_num]]   
                    node_key_one = get_node_key(hub, cycles_at_hub[cycle_num], ring)
                    node_key_two = get_node_key(hub, cycles_at_hub[other_cycle_num], other_ring)

                    arc_key = get_arc_3_key(hub, cycles_at_hub[cycle_num], 
                                            cycles_at_hub[other_cycle_num], ring, other_ring)

                    for j in 1:n_commodities
                        if ! hasbbdx(hub)
                            @constraint(m, arcflows[arc_descriptions_to_indices[arc_key],j] == 0)
                        else
                            @constraint(m, arcflows[arc_descriptions_to_indices[arc_key],j]
                                        <= adms[node_descriptions_to_indices[node_key_one]]*BIGM)

                            @constraint(m, arcflows[arc_descriptions_to_indices[arc_key],j]
                                        <= adms[node_descriptions_to_indices[node_key_two]]*BIGM)
                        end
                    end
                end
            end
        end
    end
end

for hub in type_0_nodes
    for j in 1:n_commodities
        # • The total flow of each commodity out of its origin hub (i.e., out of a Type 0 node on Type 1
        # arcs) must equal the demand for that commodity.
        if startswith(commodities[j], hub)
            # get the indices of the type-1-arcs going into hub from all rings, from all cycles
            arc_inds = []
            for c in hubs_to_cycles[string(hub)]
                for ring in 1:cycles[c]
                    arc = get_arc_1_key(hub, c, ring) 
                    arc_ind = arc_descriptions_to_indices[arc]
                    push!(arc_inds, arc_ind)
                end
            end
            # the sum of all those arcs, for the jth commodity, should be equal to the demand of that commodity
            @constraint(m, sum(arcflows[arc_ind, j] for arc_ind in arc_inds) == demands[commodities[j]])
        end

        # The total flow of each commodity into its destination hub (i.e., into a Type 0 node on Type 0
        # arcs) must equal the demand for that commodity.
        if endswith(commodities[j], hub)
            # get the indices of the 0-arcs going into hub from all rings, from all cycles
            arc_inds = []
            for c in hubs_to_cycles[string(hub)]
                for ring in 1:cycles[c]
                    arc = get_arc_0_key(hub, c, ring)
                    arc_ind = arc_descriptions_to_indices[arc]
                    push!(arc_inds, arc_ind)
                end
            end
            # the sum of all those arcs, for the jth commodity, should be equal to the demand of that commodity
            @constraint(m, sum(arcflows[arc_ind, j] for arc_ind in arc_inds) == demands[commodities[j]])
        end
    end
end

# The total flow of each commodity into each Type 1 node must equal the total flow of that
# commodity out of that Type 1 node

# these two lists are for the cost function
inds_for_ring_switching_arcs = []
inds_for_entering_or_leaving_network = []
arcflow_type_2_indices = []
unit_cost_arcflow_factors = []

for (hub_i, hub) in collect(enumerate(type_0_nodes))
    cycles_at_hub = hubs_to_cycles[hub]
    for (cycle_i, cycle) in collect(enumerate(cycles_at_hub))
        for ring in 1:cycles[cycle]
            arc_inds_in, arc_inds_out = [], []
            # get everything into it (type 1, 2, and 3 arc)

            # IN: type 1 arc (city to ring)
            arc = get_arc_1_key(hub, cycle, ring)
            push!(arc_inds_in, arc_descriptions_to_indices[arc])  
            push!(inds_for_entering_or_leaving_network, arc_descriptions_to_indices[arc])

            # OUT: type 0 arc (ring to city)
            arc = get_arc_0_key(hub, cycle, ring)
            push!(arc_inds_out, arc_descriptions_to_indices[arc]) 
            push!(inds_for_entering_or_leaving_network, arc_descriptions_to_indices[arc])

            # type 2 arcs (from an adjacent hub, but on the same ring)
            hub_index_in_cycle = search(cycle, hub)[1]
            if hub_index_in_cycle == 1
                neighboring_hub_left = cycle[end]
            else
                neighboring_hub_left = cycle[hub_index_in_cycle - 1]
            end


            if hub_index_in_cycle == length(cycle)
                neighboring_hub_right = cycle[1]
            else
                neighboring_hub_right = cycle[hub_index_in_cycle + 1]
            end


            # IN: type 2 arc, neighbor left
            arc = get_arc_2_key(neighboring_hub_left, hub, cycle, ring)
            push!(arc_inds_in, arc_descriptions_to_indices[arc]) 
            push!(arcflow_type_2_indices, arc_descriptions_to_indices[arc])
            push!(unit_cost_arcflow_factors, getflowcost(neighboring_hub_left*hub))

            # OUT: type 2 arc, neighbor left
            arc = get_arc_2_key(hub, neighboring_hub_left, cycle, ring)
            push!(arc_inds_out, arc_descriptions_to_indices[arc]) 
            push!(arcflow_type_2_indices, arc_descriptions_to_indices[arc])
            push!(unit_cost_arcflow_factors, getflowcost(hub*neighboring_hub_left))            

            # IN: type 2 arc, neighbor right
            arc = get_arc_2_key(neighboring_hub_right, hub, cycle, ring)
            push!(arc_inds_in, arc_descriptions_to_indices[arc]) 
            push!(arcflow_type_2_indices, arc_descriptions_to_indices[arc])
            push!(unit_cost_arcflow_factors, getflowcost(neighboring_hub_right*hub))

            # OUT: type 2 arc, neighbor right
            arc = get_arc_2_key(hub, neighboring_hub_right, cycle, ring)
            push!(arc_inds_out, arc_descriptions_to_indices[arc]) 
            push!(arcflow_type_2_indices, arc_descriptions_to_indices[arc])
            push!(unit_cost_arcflow_factors, getflowcost(hub*neighboring_hub_right))


            # type 3 arcs (switching rings) - only if BBDX installed
            for other_ring_same_cycle in 1:cycles[cycle]
                if ring == other_ring_same_cycle
                    continue
                end

                # IN: in-cycle in-transfers: across rings
                arc = get_arc_3_key(hub, cycle, cycle, other_ring_same_cycle, ring)
                push!(arc_inds_in, arc_descriptions_to_indices[arc])
                push!(inds_for_ring_switching_arcs, arc_descriptions_to_indices[arc])

                # OUT: in-cycle in-transfers: from this ring to other ones
                get_arc_3_key(hub, cycle, cycle, ring, other_ring_same_cycle)
                arc = get_arc_3_key(hub, cycle, cycle, ring, other_ring_same_cycle)
                push!(arc_inds_out, arc_descriptions_to_indices[arc])
                push!(inds_for_ring_switching_arcs, arc_descriptions_to_indices[arc])

            end

            # out-of-cycle in-transfers
            for other_cycle_num in append!(collect(1:cycle_i-1), collect(cycle_i+1:length(cycles_at_hub)))
                for other_ring in 1:cycles[cycles_at_hub[other_cycle_num]]

                    # IN: out-of-cycle in-transfers: across rings
                    arc = get_arc_3_key(hub, cycles_at_hub[other_cycle_num], cycle, other_ring, ring)
                    push!(arc_inds_in, arc_descriptions_to_indices[arc])
                    push!(inds_for_ring_switching_arcs, arc_descriptions_to_indices[arc])

                    # OUT: out-of-cycle in-transfers: from this ring to other ones
                    get_arc_3_key(hub, cycle, cycles_at_hub[other_cycle_num], ring, other_ring)
                    arc = get_arc_3_key(hub, cycle, cycles_at_hub[other_cycle_num], ring, other_ring)
                    push!(arc_inds_out, arc_descriptions_to_indices[arc])
                    push!(inds_for_ring_switching_arcs, arc_descriptions_to_indices[arc])
                end
            end

            @constraint(m,[j=1:n_commodities], sum(arcflows[i,j] for i in arc_inds_in)
                                                    == sum(arcflows[k,j] for k in arc_inds_out))
        end
    end
end

all_adm_transfer_inds = vcat(inds_for_ring_switching_arcs, inds_for_entering_or_leaving_network);
num_arcs_type_2 = length(arcflow_type_2_indices)

# the price of installing ADMs plus all the data that flows over them
@objective(m, Min, 1e6*sum(adms[i] for i in 1:n_type_1_nodes) +
                        2*sum(arcflows[i,j] for j in 1:n_commodities for i in all_adm_transfer_inds) +
                sum(arcflows[arcflow_type_2_indices[i],j]*unit_cost_arcflow_factors[i] for j in 1:n_commodities 
                    for i in 1:num_arcs_type_2));

solve(m)


adm_s = getvalue(adms)
af = getvalue(arcflows);

for (i,adm) in collect(enumerate(adm_s))
    if adm > 0.0001
        println(node_indices_to_descriptions[i])
    end
end

