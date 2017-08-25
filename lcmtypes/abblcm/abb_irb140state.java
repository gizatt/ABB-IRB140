/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package abblcm;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class abb_irb140state implements lcm.lcm.LCMEncodable
{
    public long utime;
    public abblcm.abb_irb140joints joints;
    public abblcm.abb_irb140cartesian cartesian;
    public abblcm.abb_irb140ftsensor force_torque;
 
    public abb_irb140state()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xc47badbba6944124L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(abblcm.abb_irb140state.class))
            return 0L;
 
        classes.add(abblcm.abb_irb140state.class);
        long hash = LCM_FINGERPRINT_BASE
             + abblcm.abb_irb140joints._hashRecursive(classes)
             + abblcm.abb_irb140cartesian._hashRecursive(classes)
             + abblcm.abb_irb140ftsensor._hashRecursive(classes)
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
 
        this.joints._encodeRecursive(outs); 
 
        this.cartesian._encodeRecursive(outs); 
 
        this.force_torque._encodeRecursive(outs); 
 
    }
 
    public abb_irb140state(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public abb_irb140state(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static abblcm.abb_irb140state _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        abblcm.abb_irb140state o = new abblcm.abb_irb140state();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.joints = abblcm.abb_irb140joints._decodeRecursiveFactory(ins);
 
        this.cartesian = abblcm.abb_irb140cartesian._decodeRecursiveFactory(ins);
 
        this.force_torque = abblcm.abb_irb140ftsensor._decodeRecursiveFactory(ins);
 
    }
 
    public abblcm.abb_irb140state copy()
    {
        abblcm.abb_irb140state outobj = new abblcm.abb_irb140state();
        outobj.utime = this.utime;
 
        outobj.joints = this.joints.copy();
 
        outobj.cartesian = this.cartesian.copy();
 
        outobj.force_torque = this.force_torque.copy();
 
        return outobj;
    }
 
}

