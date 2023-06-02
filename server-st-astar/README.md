# server-st-astar

Build
*****

<pre><code>mkdir build
cd build
cmake ..
make -j$(nproc)
</code></pre>

Run Examples
************

Server startup.
<pre><code>./server -i ../maps/sorters2.yaml -o output.yaml
</code></pre>

Oneshot algorithm start.
<pre><code>./main_st_a_star -i ../maps/sorters2.yaml -o output.yaml
</code></pre>
