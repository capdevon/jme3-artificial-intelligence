package com.examples.terrain;

import java.nio.FloatBuffer;
import java.util.logging.Logger;

import com.jme3.math.Vector3f;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.noise.Basis;
import com.jme3.terrain.noise.ShaderUtils;
import com.jme3.terrain.noise.basis.FilteredBasis;
import com.jme3.terrain.noise.filter.IterativeFilter;
import com.jme3.terrain.noise.filter.OptimizedErode;
import com.jme3.terrain.noise.filter.PerturbFilter;
import com.jme3.terrain.noise.filter.SmoothFilter;
import com.jme3.terrain.noise.fractal.FractalSum;
import com.jme3.terrain.noise.modulator.NoiseModulator;

/**
 *
 * @author capdevon
 */
public class FractalHeightMap extends AbstractHeightMap {

    private static final Logger logger = Logger.getLogger(FractalHeightMap.class.getName());

    private Basis noise;

    /**
     *
     * @param heightScale
     * @param totalSize
     */
    public FractalHeightMap(float heightScale, int totalSize) {
        this(createDefaultBasis(), heightScale, totalSize);
    }

    /**
     *
     * @param noise
     * @param heightScale
     * @param totalSize
     */
    public FractalHeightMap(Basis noise, float heightScale, int totalSize) {
        this.noise = noise;
        this.heightScale = heightScale;
        this.size = totalSize;
        load();
    }

    public float[] generateHeightMap(Vector3f location) {
        int xPos = (int) location.getX();
        int zPos = (int) location.getZ();
        float sx = xPos * (size - 1);
        float sy = zPos * (size - 1);
        float base = 0;

        FloatBuffer buffer = noise.getBuffer(sx, sy, base, size);
        float[] arr = buffer.array();
        for (int i = 0; i < arr.length; i++) {
            arr[i] = arr[i] * heightScale;
        }
        return buffer.array();
    }

    @Override
    public boolean load() {
        // clean up data if needed.
        if (null != heightData) {
            unloadHeightMap();
        }
        heightData = generateHeightMap(new Vector3f(0, 0, 0));

        logger.info("Created FractalHeightMap");

        return true;
    }

    private static FilteredBasis createDefaultBasis() {
        FractalSum base = new FractalSum();
        base.setRoughness(0.7f);
        base.setFrequency(1.0f);
        base.setAmplitude(1.0f);
        base.setLacunarity(2.12f);
        base.setOctaves(8);
        base.setScale(0.02125f);
        base.addModulator(new NoiseModulator() {
            @Override
            public float value(float... in) {
                return ShaderUtils.clamp(in[0] * 0.5f + 0.5f, 0, 1);
            }
        });

        PerturbFilter perturb = new PerturbFilter();
        perturb.setMagnitude(0.119f);

        OptimizedErode therm = new OptimizedErode();
        therm.setRadius(5);
        therm.setTalus(0.011f);

        SmoothFilter smooth = new SmoothFilter();
        smooth.setRadius(1);
        smooth.setEffect(0.7f);

        IterativeFilter iterate = new IterativeFilter();
        iterate.addPreFilter(perturb);
        iterate.addPostFilter(smooth);
        iterate.setFilter(therm);
        iterate.setIterations(1);

        FilteredBasis ground = new FilteredBasis(base);
        ground.addPreFilter(iterate);
        return ground;
    }

}
