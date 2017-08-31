/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package abblcm;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class abb_irb140joints implements lcm.lcm.LCMEncodable
{
    public long utime;
    public double pos[];
    public double vel[];
 
    public abb_irb140joints()
    {
        pos = new double[6];
        vel = new double[6];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xdb4f922fdc668f51L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(abblcm.abb_irb140joints.class))
            return 0L;
 
        classes.add(abblcm.abb_irb140joints.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        for (int a = 0; a < 6; a++) {
            outs.writeDouble(this.pos[a]); 
        }
 
        for (int a = 0; a < 6; a++) {
            outs.writeDouble(this.vel[a]); 
        }
 
    }
 
    public abb_irb140joints(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public abb_irb140joints(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static abblcm.abb_irb140joints _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        abblcm.abb_irb140joints o = new abblcm.abb_irb140joints();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.pos = new double[(int) 6];
        for (int a = 0; a < 6; a++) {
            this.pos[a] = ins.readDouble();
        }
 
        this.vel = new double[(int) 6];
        for (int a = 0; a < 6; a++) {
            this.vel[a] = ins.readDouble();
        }
 
    }
 
    public abblcm.abb_irb140joints copy()
    {
        abblcm.abb_irb140joints outobj = new abblcm.abb_irb140joints();
        outobj.utime = this.utime;
 
        outobj.pos = new double[(int) 6];
        System.arraycopy(this.pos, 0, outobj.pos, 0, 6); 
        outobj.vel = new double[(int) 6];
        System.arraycopy(this.vel, 0, outobj.vel, 0, 6); 
        return outobj;
    }
 
}
