// GPU GA* program

#pragma OPENCL EXTENSION cl_amd_printf : enable

#if 0
#pragma OPENCL EXTENSION cl_khr_int64_base_atomics : enable
#endif

#define SQRT2 1.41421356237f

// ----- Types ----------------------------------------------------------------
typedef struct {
    uint  first;
    float second;
} uint_float;

typedef struct {  // Depending on use case...
    uint  closed; // only one of the first two members is used here
    uint  node;   // the other one gives padding for memory alignment
    float totalCost;
    uint  predecessor;
} Info;

struct TimeRange{
    float start;
    float end;
};

bool inRange(float aStart, float aEnd, float start, float end){
    if ((start == end) || (aStart == aEnd))
        return false;

    return (((aStart >= start) && (aStart <= end)) ||
            ((aEnd >= start) && (aEnd <= end)) ||
            ((aEnd >= end) && (aStart <= start)));
}

// ----- Helper ---------------------------------------------------------------
uint_float _read_heap(__global uint_float *open, size_t index) {
    return open[index];
}

void _write_heap(__global uint_float *open, size_t index, uint_float value) {
    open[index] = value;
}

// ----- OpenList functions ---------------------------------------------------
uint top(__global uint_float *open) {
    return open[0].first;
}

void push(__global uint_float *open, size_t *size, uint value, float cost) {
    size_t index = (*size)++;

    while (index > 0) {
        size_t parent = (index - 1) / 2;

        uint_float pValue = _read_heap(open, parent);
        if (cost < pValue.second) {
            _write_heap(open, index, pValue);
            index = parent;
        } else
            break;
    }

    _write_heap(open, index, (uint_float){value, cost});
}

void pop(__global uint_float *open, size_t *size) {
    uint_float value = _read_heap(open, --(*size));
    size_t     index = 0;

    while (index < *size / 2) {
        size_t child = index * 2 + 1;

        uint_float cValue = _read_heap(open, child);
        if (child + 1 < *size) {
            uint_float c1Value = _read_heap(open, child + 1);

            if (c1Value.second < cValue.second) {
                ++child;
                cValue = c1Value;
            }
        }

        if (cValue.second < value.second) {
            _write_heap(open, index, cValue);
            index = child;
        } else
            break;
    }

    _write_heap(open, index, value);
}

// ----- Kernels --------------------------------------------------------------
__kernel void clearList(__global uint *list, const ulong size) {
    if (get_global_id(0) < size)
        list[get_global_id(0)] = 0;
}

__kernel void extractAndExpand(__global const uint_float *edges,            // destination index, stepCost
                                        const ulong       edgesSize,
                               __global const uint2      *adjacencyMap,     // edges_begin, edges_end
                                        const ulong       adjacencyMapSize,
                                        const ulong       numberOfQueues,   // provides offset ...
                                        const ulong       sizeOfAQueue,     // provides offset ...
                                        const uint        destination,      // destination index
                               __global       uint_float *openLists,        // aka "Q" priority queues
                               __global       uint       *openSizes,
                               __global       Info       *info,             // closed list, see members at the top
                               __global       Info       *slistChunks,      // "S" list, divided into chunks
                               __global       uint       *slistSizes,
                                        const ulong       slistChunkSize,
                               __global       uint       *returnCode,
                               __global const float2     *TOT,
                                        const uint       width,
                                        const uint       height)
{
    // Parallel for each queue (one dimensional)
    const size_t GID = get_global_id(0);

    if (GID >= numberOfQueues)
        return;

    __global uint_float *openList = openLists + GID * sizeOfAQueue;
    __global Info       *slist = slistChunks + GID * slistChunkSize;

    size_t openSize = openSizes[GID]; // read open list size
    uint   slistSize = 0;

    if (openSize == 0) {
        atomic_min(returnCode, 2);
        return; // failure: no path found!
    }

    const uint current = top(openList);
    pop(openList, &openSize);

    if (current == destination) {
        atomic_min(returnCode, 0);
        return; // success: path found!
    }

    // In this algorithm, "closed" means already added to open list.
    // --> nothing to do here.
    const Info currentInfo = info[current];

    // ��������� ���� �������
    const uint2 edgeRange = adjacencyMap[current];
    bool dynamicObstacle = false;
    for (uint edge = edgeRange.x; edge != edgeRange.y; ++edge) {
        const uint  nbNode     = edges[edge].first;
        const float nbStepCost = edges[edge].second;

        const float nbTotalCost = currentInfo.totalCost + nbStepCost;

        // TODO: rewrite for a vector: cycle or something
        float2 timeRange[5] = {TOT[nbNode*5], *(&TOT[nbNode*5]+1), *(&TOT[nbNode*5]+2), *(&TOT[nbNode*5]+3), *(&TOT[nbNode*5]+4)};

        printf("nbNode: %d, y %d, x %d\n", nbNode, nbNode / width, nbNode % width);

        // Check TOT
        for (int iter=0; iter<5; ++iter){
            if (inRange(nbTotalCost, nbTotalCost + 1, timeRange[iter].x, timeRange[iter].y)){
                printf("dynamicObstacle: y: %d, x: %d, tx: %.2f, ty: %.2f, nbtx: %.2f, nbty: %.2f\n",
                    nbNode / width, nbNode % width, timeRange[iter].x, timeRange[iter].y, nbTotalCost, nbTotalCost+1);
                dynamicObstacle = true;
                break;
            }
        }

        if (!dynamicObstacle) {
            printf("AddedMove: y: %d, x: %d, tx: %.2f, ty: %.2f\n",
                nbNode / width, nbNode % width, nbTotalCost, nbTotalCost+1);
            slist[slistSize++] = (Info){0, nbNode, nbTotalCost, current};
        }
    }

    if (dynamicObstacle){
        int xy = current % (width*height);
        int time = current / (width*height);
        float2 timeRange[5] = {TOT[xy*5], *(&TOT[xy*5]+1), *(&TOT[xy*5]+2), *(&TOT[xy*5]+3), *(&TOT[xy*5]+4)};

        //printf("nbNode: %d\n", xy);
        //printf("timeRange.y: %.2f\n", timeRange.y);

        bool cantStop = false;
        // Check TOT
        for (int iter=0; iter<5; ++iter){
            if (inRange(currentInfo.totalCost, currentInfo.totalCost + 1, timeRange[iter].x, timeRange[iter].y)){
                printf("Wait: dynamicObstacle: y: %d, x: %d, tx: %.2f, ty: %.2f, currtx: %.2f, currty: %.2f\n",
                    xy / width, xy % width, timeRange[iter].x, timeRange[iter].y, currentInfo.totalCost, currentInfo.totalCost+1);
                cantStop = true;
                break;
            }
        }
        if (!cantStop){
            slist[slistSize++] = (Info){0, current+(width*height), currentInfo.totalCost + 1, current};
            printf("AddedWaiting: y: %d, x: %d, time: %d, tx: %.2f, ty: %.2f\n",
            xy / width, xy % width, time, currentInfo.totalCost, currentInfo.totalCost + 1);
        }
    }

    // Write back new list sizes
    openSizes[GID] = (uint) openSize;
    slistSizes[GID] = slistSize;

    atomic_min(returnCode, 1); // still running...
}

__kernel void duplicateDetection(         const ulong       numberOfQueues,   // provides offset ...
                                 __global       Info       *info,             // closed list, see members at the top
                                 __global       Info       *slistChunks,      // "S" list, divided into chunks
                                 __global       uint       *slistSizes,
                                          const ulong       slistChunkSize,   // equals "tlistChunkSize" as well
                                 __global       Info       *tlistChunks,      // "T" list, divided into chunks
                                 __global       uint       *tlistSizes,
                                 __global       uint       *hashTable,
                                          const ulong       hashTableSize)
{
    // Parallel for each element in S-list (two dimensional)
    const uint2 GID = {get_global_id(0), get_global_id(1)};

    if (GID.x >= numberOfQueues || GID.y >= slistChunkSize)
        return;

    __global Info *slist = slistChunks + GID.x * slistChunkSize;
    uint slistSize = slistSizes[GID.x];

    if (GID.y >= slistSize)
        return;

    const Info current = slist[GID.y];
    const Info nodeInfo = info[current.node];

    // In this algorithm, "closed" means already added to open list.
    if (nodeInfo.closed == 1 && nodeInfo.totalCost < current.totalCost)
        return; // better candidate already in open list

    // Deduplication with hashing
    const uint hash = current.node % hashTableSize; // TODO: size +/- 1 for better collisions ?
    const uint old = atomic_xchg(hashTable + hash, current.node);

    if (old == current.node)
        return; // node has already been added

    // TODO: There is some searching in the script. Should we add that? I don't see the point...

    __global Info *tlist = tlistChunks + GID.x * slistChunkSize;
    const uint index = atomic_inc(tlistSizes + GID.x);
    tlist[index] = current;
}

__kernel void compactTList(         const ulong       numberOfQueues,   // provides offset ...
                           __global const Info       *tlistChunks,      // "T" list, divided into chunks
                           __global const uint       *tlistSizes,
                                    const ulong       tlistChunkSize,
                           __global       uint       *exclusiveSums,
                           __global       Info       *tlistCompacted,
                           __global       uint       *tlistCompactedSize)
{
    // Parallel for each element in T-list (two dimensional)
    const uint2 GID = {get_global_id(0), get_global_id(1)};

    if (GID.x >= numberOfQueues || GID.y >= tlistChunkSize)
        return;

    __global const Info *tlist = tlistChunks + GID.x * tlistChunkSize;
    const uint tlistSize = tlistSizes[GID.x];
    const uint index = exclusiveSums[GID.x];

    // Store size
    if (GID.x == numberOfQueues - 1 && GID.y == 0)
        *tlistCompactedSize = index + tlistSize;

    if (GID.y >= tlistSize)
        return;

    tlistCompacted[index + GID.y] = tlist[GID.y];
}

// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance
float heuristic(int4 source, int4 destination) {
    const int dx = abs(destination.x - source.x);
    const int dy = abs(destination.y - source.y);
    return (dx + dy) + (SQRT2 - 2) * min(dx, dy);
}

__kernel void computeAndPushBack(__global const int4      *nodes,            // x, y, t
                                          const ulong       nodesSize,
                                          const ulong       numberOfQueues,   // provides offset ...
                                          const ulong       sizeOfAQueue,     // provides offset ...
                                          const uint        destination,      // destination index
                                 __global       uint_float *openLists,        // aka "Q" priority queues
                                 __global       uint       *openSizes,
                                 __global       Info       *info,             // closed list, see members at the top
                                 __global const Info       *tlistCompacted,   // "T" list, compacted!
                                 __global const uint       *tlistCompactedSize,
                                 __global const uint       *queueRotation)
{
    // Parallel for each queue (one dimensional)
    const size_t GID = get_global_id(0);

    if (GID >= numberOfQueues)
        return;

    const int4 destNode = nodes[destination];

    const size_t openIndex = (GID + *queueRotation) % numberOfQueues;
    __global uint_float *openList = openLists + openIndex * sizeOfAQueue;
    size_t openSize = openSizes[openIndex]; // read open list size

    const size_t tlistSize = *tlistCompactedSize;
    for (size_t i = GID; i < tlistSize; i += numberOfQueues) {
        // FIXME: Drop node if open list is full!
        // --> Should be fixed with using compacted tlist!
        if (openSize == sizeOfAQueue)
            break;

        const Info current = tlistCompacted[i];

#if 0
        // Replaced: This idea doesn't seem to work for some reason...

        // In this algorithm, "closed" means already added to open list.
        info[current.node].closed = 1; // close node
        
        // Update totalCost and predecessor as one 64 bit transaction.
        ulong *currentCostPred = (ulong *) &current.totalCost;
        __global ulong *infoCostPred = (__global ulong *) &info[current.node].totalCost;
        ulong oldCostPred = atom_xchg(infoCostPred, *currentCostPred);
        float *oldCost = (float *) &oldCostPred;

        // assert: current.totalCost > 0.0f
        while (*oldCost != 0.0f && *oldCost < current.totalCost) {
            // The old entry was better. Swap back!
            *currentCostPred = oldCostPred;
            oldCostPred = atom_xchg(infoCostPred, *currentCostPred);
        }
#else
        // FIXME: There is still a bug here, a data race on duplicate nodes!
        Info nodeInfo = info[current.node];
        if (nodeInfo.totalCost == 0.0f || current.totalCost < nodeInfo.totalCost) {
            nodeInfo.totalCost = current.totalCost;
            nodeInfo.predecessor = current.predecessor;
        }

        // In this algorithm, "closed" means already added to open list.
        nodeInfo.closed = 1; // close node

        info[current.node] = nodeInfo; // write back
#endif

        float h = heuristic(nodes[current.node], destNode);
        push(openList, &openSize, current.node, current.totalCost + h);
    }

    // Write back new list size
    openSizes[openIndex] = (uint) openSize;
}
