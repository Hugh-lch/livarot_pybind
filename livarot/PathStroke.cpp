// SPDX-License-Identifier: GPL-2.0-or-later
/** @file
 * TODO: insert short description here
 *//*
 * Authors:
 * see git history
 * Fred
 *
 * Copyright (C) 2018 Authors
 * Released under GNU GPL v2+, read the file 'COPYING' for more information.
 */

#include "Path.h"
#include "Shape.h"
#include "geom.h"
#include <2geom/transforms.h>
#include <2geom/pathvector.h>
#include <2geom/sweeper.h>
#include <random>

/*
 * stroking polylines into a Shape instance
 * grunt work.
 * if the goal is to raster the stroke, polyline stroke->polygon->uncrossed polygon->raster is grossly
 * inefficient (but reuse the intersector, so that's what a lazy programmer like me does). the correct way would be
 * to set up a supersampled buffer, raster each polyline stroke's part (one part per segment in the polyline, plus 
 * each join) because these are all convex polygons, then transform in alpha values
 */

// until i find something better
static Geom::Point StrokeNormalize(const Geom::Point value) {
    double length = L2(value); 
    if ( length < 0.0000001 ) { 
        return Geom::Point(0, 0);
    } else { 
        return value/length; 
    }
}

// faster version if length is known
static Geom::Point StrokeNormalize(const Geom::Point value, double length) {
    if ( length < 0.0000001 ) { 
        return Geom::Point(0, 0);
    } else { 
        return value/length; 
    }
}

void Path::Stroke(Shape *dest, bool doClose, double width, JoinType join,
        ButtType butt, double miter, bool justAdd)
{
    if (dest == nullptr) {
        return;
    }

    if (justAdd == false) {
        dest->Reset(3 * pts.size(), 3 * pts.size());
    }

    dest->MakeBackData(false);

    int lastM = 0;
    while (lastM < int(pts.size())) {

        int lastP = lastM + 1;
        while (lastP < int(pts.size()) // select one subpath
                && (pts[lastP].isMoveTo == polyline_lineto
                    || pts[lastP].isMoveTo == polyline_forced))
        {
            lastP++;
        }

        if ( lastP > lastM+1 ) {
            Geom::Point sbStart = pts[lastM].p;
            Geom::Point sbEnd = pts[lastP - 1].p;
            // if ( pts[lastP - 1].closed ) { // this is correct, but this bugs text rendering (doesn't close text stroke)...
            if ( Geom::LInfty(sbEnd-sbStart) < 0.00001 ) {       // why close lines that shouldn't be closed?
                // ah I see, because close is defined here for
                // a whole path and should be defined per subpath.
                // debut==fin => ferme (on devrait garder un element pour les close(), mais tant pis)
                DoStroke(lastM, lastP - lastM, dest, true, width, join, butt, miter, true);
            } else {
                DoStroke(lastM, lastP - lastM, dest, doClose, width, join, butt, miter, true);
            }
        } else if (butt == butt_round) {       // special case: zero length round butt is a circle
            int last[2] = { -1, -1 };
            Geom::Point dir;
            dir[0] = 1;
            dir[1] = 0;
            Geom::Point pos = pts[lastM].p;
            DoButt(dest, width, butt, pos, dir, last[RIGHT], last[LEFT]);
            int end[2];
            dir = -dir;
            DoButt(dest, width, butt, pos, dir, end[LEFT], end[RIGHT]);
            dest->AddEdge (end[LEFT], last[LEFT]);
            dest->AddEdge (last[RIGHT], end[RIGHT]);
        }
        lastM = lastP;
    }
}

void Path::DoStroke(int off, int N, Shape *dest, bool doClose, double width, JoinType join,
		    ButtType butt, double miter, bool /*justAdd*/)
{
    if (N <= 1) {
        return;
    }

    Geom::Point prevP, nextP;
    int prevI, nextI;
    int upTo;

    int curI = 0;
    Geom::Point curP = pts[off].p;

    if (doClose) {

        prevI = N - 1;
        while (prevI > 0) {
            prevP = pts[off + prevI].p;
            Geom::Point diff = curP - prevP;
            double dist = dot(diff, diff);
            if (dist > 0.001) {
                break;
            }
            prevI--;
        }
        if (prevI <= 0) {
            return;
        }
        upTo = prevI;

    } else {

        prevP = curP;
        prevI = curI;
        upTo = N - 1;
    }

    {
        nextI = 1;
        while (nextI <= upTo) {
            nextP = pts[off + nextI].p;
            Geom::Point diff = curP - nextP;
            double dist = dot(diff, diff);
            if (dist > 0.0) { // more tolerance for the first distance, to give the cap the right direction
                break;
            }
            nextI++;
        }
        if (nextI > upTo) {
            if (butt == butt_round) {  // special case: zero length round butt is a circle
                int last[2] = { -1, -1 };
                Geom::Point dir;
                dir[0] = 1;
                dir[1] = 0;
                DoButt(dest, width, butt, curP, dir, last[RIGHT], last[LEFT]);
                int end[2];
                dir = -dir;
                DoButt(dest, width, butt, curP, dir, end[LEFT], end[RIGHT]);
                dest->AddEdge (end[LEFT], last[LEFT]);
                dest->AddEdge (last[RIGHT], end[RIGHT]);
            }
            return;
        }
    }

    int start[2] = { -1, -1 };
    int last[2] = { -1, -1 };
    Geom::Point prevD = curP - prevP;
    Geom::Point nextD = nextP - curP;
    double prevLe = Geom::L2(prevD);
    double nextLe = Geom::L2(nextD);
    prevD = StrokeNormalize(prevD, prevLe);
    nextD = StrokeNormalize(nextD, nextLe);

    if (doClose) {
        DoJoin(dest,  width, join, curP, prevD, nextD, miter, prevLe, nextLe, start, last);
    } else {
        nextD = -nextD;
        DoButt(dest,  width, butt, curP, nextD, last[RIGHT], last[LEFT]);
        nextD = -nextD;
    }

    do {
        prevP = curP;
        prevI = curI;
        curP = nextP;
        curI = nextI;
        prevD = nextD;
        prevLe = nextLe;
        nextI++;
        while (nextI <= upTo) {
            nextP = pts[off + nextI].p;
            Geom::Point diff = curP - nextP;
            double dist = dot(diff, diff);
            if (dist > 0.001 || (nextI == upTo && dist > 0.0)) { // more tolerance for the last distance too, for the right cap direction
                break;
            }
            nextI++;
        }
        if (nextI > upTo) {
            break;
        }

        nextD = nextP - curP;
        nextLe = Geom::L2(nextD);
        nextD = StrokeNormalize(nextD, nextLe);
        int nSt[2] = { -1, -1 };
        int nEn[2] = { -1, -1 };
        DoJoin(dest, width, join, curP, prevD, nextD, miter, prevLe, nextLe, nSt, nEn);
        dest->AddEdge(nSt[LEFT], last[LEFT]);
        last[LEFT] = nEn[LEFT];
        dest->AddEdge(last[RIGHT], nSt[RIGHT]);
        last[RIGHT] = nEn[RIGHT];
    } while (nextI <= upTo);

    if (doClose) {
        /*		prevP=curP;
                        prevI=curI;
                        curP=nextP;
                        curI=nextI;
                        prevD=nextD;*/
        nextP = pts[off].p;

        nextD = nextP - curP;
        nextLe = Geom::L2(nextD);
        nextD = StrokeNormalize(nextD, nextLe);
        int nSt[2] = { -1, -1 };
        int nEn[2] = { -1, -1 };
        DoJoin(dest,  width, join, curP, prevD, nextD, miter, prevLe, nextLe, nSt, nEn);
        dest->AddEdge (nSt[LEFT], last[LEFT]);
        last[LEFT] = nEn[LEFT];
        dest->AddEdge (last[RIGHT], nSt[RIGHT]);
        last[RIGHT] = nEn[RIGHT];

        dest->AddEdge (start[LEFT], last[LEFT]);
        dest->AddEdge (last[RIGHT], start[RIGHT]);

    } else {

        int end[2];
        DoButt (dest,  width, butt, curP, prevD, end[LEFT], end[RIGHT]);
        dest->AddEdge (end[LEFT], last[LEFT]);
        dest->AddEdge (last[RIGHT], end[RIGHT]);
    }
}


void Path::DoButt(Shape *dest, double width, ButtType butt, Geom::Point pos, Geom::Point dir,
        int &leftNo, int &rightNo)
{
    Geom::Point nor;
    nor = dir.ccw();

    if (butt == butt_square)
    {
        Geom::Point x;
        x = pos + width * dir + width * nor;
        int bleftNo = dest->AddPoint (x);
        x = pos + width * dir - width * nor;
        int brightNo = dest->AddPoint (x);
        x = pos + width * nor;
        leftNo = dest->AddPoint (x);
        x = pos - width * nor;
        rightNo = dest->AddPoint (x);
        dest->AddEdge (rightNo, brightNo);
        dest->AddEdge (brightNo, bleftNo);
        dest->AddEdge (bleftNo, leftNo);
    }
    else if (butt == butt_pointy)
    {
        leftNo = dest->AddPoint (pos + width * nor);
        rightNo = dest->AddPoint (pos - width * nor);
        int mid = dest->AddPoint (pos + width * dir);
        dest->AddEdge (rightNo, mid);
        dest->AddEdge (mid, leftNo);
    }
    else if (butt == butt_round)
    {
        const Geom::Point sx = pos + width * nor;
        const Geom::Point ex = pos - width * nor;
        leftNo = dest->AddPoint (sx);
        rightNo = dest->AddPoint (ex);

        RecRound (dest, rightNo, leftNo, ex, sx, -nor, nor, pos, width);
    }
    else
    {
        leftNo = dest->AddPoint (pos + width * nor);
        rightNo = dest->AddPoint (pos - width * nor);
        dest->AddEdge (rightNo, leftNo);
    }
}


void Path::DoJoin (Shape *dest, double width, JoinType join, Geom::Point pos, Geom::Point prev,
        Geom::Point next, double miter, double /*prevL*/, double /*nextL*/,
        int *stNo, int *enNo)
{
    Geom::Point pnor = prev.ccw();
    Geom::Point nnor = next.ccw();
    double angSi = cross(prev, next);

    /* FIXED: this special case caused bug 1028953 */
    if (angSi > -0.0001 && angSi < 0.0001) {
        double angCo = dot (prev, next);
        if (angCo > 0.9999) {
            // tout droit
            stNo[LEFT] = enNo[LEFT] = dest->AddPoint(pos + width * pnor);
            stNo[RIGHT] = enNo[RIGHT] = dest->AddPoint(pos - width * pnor);
        } else {
            // demi-tour
            const Geom::Point sx = pos + width * pnor;
            const Geom::Point ex = pos - width * pnor;
            stNo[LEFT] = enNo[RIGHT] = dest->AddPoint (sx);
            stNo[RIGHT] = enNo[LEFT] = dest->AddPoint (ex);
            if (join == join_round) {
                RecRound (dest, enNo[LEFT], stNo[LEFT], ex, sx, -pnor, pnor, pos, width);
                dest->AddEdge(stNo[RIGHT], enNo[RIGHT]);
            } else {
                dest->AddEdge(enNo[LEFT], stNo[LEFT]);
                dest->AddEdge(stNo[RIGHT], enNo[RIGHT]);	// two times because both are crossing each other
            }
        }
        return;
    }

    if (angSi < 0) {
        int midNo = dest->AddPoint(pos);
        stNo[LEFT] = dest->AddPoint(pos + width * pnor);
        enNo[LEFT] = dest->AddPoint(pos + width * nnor);
        dest->AddEdge(enNo[LEFT], midNo);
        dest->AddEdge(midNo, stNo[LEFT]);

        if (join == join_pointy) {

            stNo[RIGHT] = dest->AddPoint(pos - width * pnor);
            enNo[RIGHT] = dest->AddPoint(pos - width * nnor);

            const Geom::Point biss = StrokeNormalize(prev - next);
            double c2 = dot(biss, nnor);
            double l = width / c2;
            double emiter = width * c2;
            if (emiter < miter) {
                emiter = miter;
            }

            if (fabs(l) < miter) {
                int const n = dest->AddPoint(pos - l * biss);
                dest->AddEdge(stNo[RIGHT], n);
                dest->AddEdge(n, enNo[RIGHT]);
            } else {
                dest->AddEdge(stNo[RIGHT], enNo[RIGHT]);
            }

        } else if (join == join_round) {
            Geom::Point sx = pos - width * pnor;
            stNo[RIGHT] = dest->AddPoint(sx);
            Geom::Point ex = pos - width * nnor;
            enNo[RIGHT] = dest->AddPoint(ex);

            RecRound(dest, stNo[RIGHT], enNo[RIGHT], 
                    sx, ex, -pnor, -nnor, pos, width);

        } else {
            stNo[RIGHT] = dest->AddPoint(pos - width * pnor);
            enNo[RIGHT] = dest->AddPoint(pos - width * nnor);
            dest->AddEdge(stNo[RIGHT], enNo[RIGHT]);
        }

    } else {

        int midNo = dest->AddPoint(pos);
        stNo[RIGHT] = dest->AddPoint(pos - width * pnor);
        enNo[RIGHT] = dest->AddPoint(pos - width * nnor);
        dest->AddEdge(stNo[RIGHT], midNo);
        dest->AddEdge(midNo, enNo[RIGHT]);

        if (join == join_pointy) {

            stNo[LEFT] = dest->AddPoint(pos + width * pnor);
            enNo[LEFT] = dest->AddPoint(pos + width * nnor);

            const Geom::Point biss = StrokeNormalize(next - prev);
            double c2 = dot(biss, nnor);
            double l = width / c2;
            double emiter = width * c2;
            if (emiter < miter) {
                emiter = miter;
            }
            if ( fabs(l) < miter) {
                int const n = dest->AddPoint (pos + l * biss);
                dest->AddEdge (enNo[LEFT], n);
                dest->AddEdge (n, stNo[LEFT]);
            }
            else
            {
                dest->AddEdge (enNo[LEFT], stNo[LEFT]);
            }

        } else if (join == join_round) {

            Geom::Point sx = pos + width * pnor;
            stNo[LEFT] = dest->AddPoint(sx);
            Geom::Point ex = pos + width * nnor;
            enNo[LEFT] = dest->AddPoint(ex);

            RecRound(dest, enNo[LEFT], stNo[LEFT], 
                    ex, sx, nnor, pnor, pos, width);

        } else {
            stNo[LEFT] = dest->AddPoint(pos + width * pnor);
            enNo[LEFT] = dest->AddPoint(pos + width * nnor);
            dest->AddEdge(enNo[LEFT], stNo[LEFT]);
        }
    }
}

    void
Path::DoLeftJoin (Shape * dest, double width, JoinType join, Geom::Point pos,
        Geom::Point prev, Geom::Point next, double miter, double /*prevL*/, double /*nextL*/,
        int &leftStNo, int &leftEnNo,int pathID,int pieceID,double tID)
{
    Geom::Point pnor=prev.ccw();
    Geom::Point nnor=next.ccw();
    double angSi = cross(prev, next);
    if (angSi > -0.0001 && angSi < 0.0001)
    {
        double angCo = dot (prev, next);
        if (angCo > 0.9999)
        {
            // tout droit
            leftEnNo = leftStNo = dest->AddPoint (pos + width * pnor);
        }
        else
        {
            // demi-tour
            leftStNo = dest->AddPoint (pos + width * pnor);
            leftEnNo = dest->AddPoint (pos - width * pnor);
            int nEdge=dest->AddEdge (leftEnNo, leftStNo);
            if ( dest->hasBackData() ) {
                dest->ebData[nEdge].pathID=pathID;
                dest->ebData[nEdge].pieceID=pieceID;
                dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
            }
        }
        return;
    }
    if (angSi < 0)
    {
        /*		Geom::Point     biss;
                        biss.x=next.x-prev.x;
                        biss.y=next.y-prev.y;
                        double   c2=cross(next, biss);
                        double   l=width/c2;
                        double		projn=l*(dot(biss,next));
                        double		projp=-l*(dot(biss,prev));
                        if ( projp <= 0.5*prevL && projn <= 0.5*nextL ) {
                        double   x,y;
                        x=pos.x+l*biss.x;
                        y=pos.y+l*biss.y;
                        leftEnNo=leftStNo=dest->AddPoint(x,y);
                        } else {*/
        leftStNo = dest->AddPoint (pos + width * pnor);
        leftEnNo = dest->AddPoint (pos + width * nnor);
//        int midNo = dest->AddPoint (pos);
//        int nEdge=dest->AddEdge (leftEnNo, midNo);
        int nEdge=dest->AddEdge (leftEnNo, leftStNo);
        if ( dest->hasBackData() ) {
            dest->ebData[nEdge].pathID=pathID;
            dest->ebData[nEdge].pieceID=pieceID;
            dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
        }
//        nEdge=dest->AddEdge (midNo, leftStNo);
//        if ( dest->hasBackData() ) {
//            dest->ebData[nEdge].pathID=pathID;
//            dest->ebData[nEdge].pieceID=pieceID;
//            dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
//        }
        //              }
    }
    else
    {
        if (join == join_pointy)
        {
            leftStNo = dest->AddPoint (pos + width * pnor);
            leftEnNo = dest->AddPoint (pos + width * nnor);

            const Geom::Point biss = StrokeNormalize (pnor + nnor);
            double c2 = dot (biss, nnor);
            double l = width / c2;
            double emiter = width * c2;
            if (emiter < miter)
                emiter = miter;
            if (l <= emiter)
            {
                int nleftStNo = dest->AddPoint (pos + l * biss);
                int nEdge=dest->AddEdge (leftEnNo, nleftStNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
                nEdge=dest->AddEdge (nleftStNo, leftStNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
            }
            else
            {
                double s2 = cross(nnor, biss);
                double dec = (l - emiter) * c2 / s2;
                const Geom::Point tbiss=biss.ccw();

                int nleftStNo = dest->AddPoint (pos + emiter * biss + dec * tbiss);
                int nleftEnNo = dest->AddPoint (pos + emiter * biss - dec * tbiss);
                int nEdge=dest->AddEdge (nleftEnNo, nleftStNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
                nEdge=dest->AddEdge (leftEnNo, nleftEnNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
                nEdge=dest->AddEdge (nleftStNo, leftStNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
            }
        }
        else if (join == join_round)
        {
            const Geom::Point sx = pos + width * pnor;
            leftStNo = dest->AddPoint (sx);
            const Geom::Point ex = pos + width * nnor;
            leftEnNo = dest->AddPoint (ex);

            RecRound(dest, leftEnNo, leftStNo, 
                    sx, ex, pnor, nnor ,pos, width);

        }
        else
        {
            leftStNo = dest->AddPoint (pos + width * pnor);
            leftEnNo = dest->AddPoint (pos + width * nnor);
            int nEdge=dest->AddEdge (leftEnNo, leftStNo);
            if ( dest->hasBackData() ) {
                dest->ebData[nEdge].pathID=pathID;
                dest->ebData[nEdge].pieceID=pieceID;
                dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
            }
        }
    }
}
    void
Path::DoRightJoin (Shape * dest, double width, JoinType join, Geom::Point pos,
        Geom::Point prev, Geom::Point next, double miter, double /*prevL*/,
        double /*nextL*/, int &rightStNo, int &rightEnNo,int pathID,int pieceID,double tID)
{
    const Geom::Point pnor=prev.ccw();
    const Geom::Point nnor=next.ccw();
    double angSi = cross(prev, next);
    if (angSi > -0.0001 && angSi < 0.0001)
    {
        double angCo = dot (prev, next);
        if (angCo > 0.9999)
        {
            // tout droit
            rightEnNo = rightStNo = dest->AddPoint (pos - width*pnor);
        }
        else
        {
            // demi-tour
            rightEnNo = dest->AddPoint (pos + width*pnor);
            rightStNo = dest->AddPoint (pos - width*pnor);
            int nEdge=dest->AddEdge (rightStNo, rightEnNo);
            if ( dest->hasBackData() ) {
                dest->ebData[nEdge].pathID=pathID;
                dest->ebData[nEdge].pieceID=pieceID;
                dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
            }
        }
        return;
    }
    if (angSi < 0)
    {
        if (join == join_pointy)
        {
            rightStNo = dest->AddPoint (pos - width*pnor);
            rightEnNo = dest->AddPoint (pos - width*nnor);

            const Geom::Point biss = StrokeNormalize (pnor + nnor);
            double c2 = dot (biss, nnor);
            double l = width / c2;
            double emiter = width * c2;
            if (emiter < miter)
                emiter = miter;
            if (l <= emiter)
            {
                int nrightStNo = dest->AddPoint (pos - l * biss);
                int nEdge=dest->AddEdge (rightStNo, nrightStNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
                nEdge=dest->AddEdge (nrightStNo, rightEnNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
            }
            else
            {
                double s2 = cross(nnor, biss);
                double dec = (l - emiter) * c2 / s2;
                const Geom::Point tbiss=biss.ccw();

                int nrightStNo = dest->AddPoint (pos - emiter*biss - dec*tbiss);
                int nrightEnNo = dest->AddPoint (pos - emiter*biss + dec*tbiss);
                int nEdge=dest->AddEdge (rightStNo, nrightStNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
                nEdge=dest->AddEdge (nrightStNo, nrightEnNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
                nEdge=dest->AddEdge (nrightEnNo, rightEnNo);
                if ( dest->hasBackData() ) {
                    dest->ebData[nEdge].pathID=pathID;
                    dest->ebData[nEdge].pieceID=pieceID;
                    dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
                }
            }
        }
        else if (join == join_round)
        {
            const Geom::Point sx = pos - width * pnor;
            rightStNo = dest->AddPoint (sx);
            const Geom::Point ex = pos - width * nnor;
            rightEnNo = dest->AddPoint (ex);

            RecRound(dest, rightStNo, rightEnNo, 
                    sx, ex, -pnor, -nnor ,pos, width);
        }
        else
        {
            rightStNo = dest->AddPoint (pos - width * pnor);
            rightEnNo = dest->AddPoint (pos - width * nnor);
            int nEdge=dest->AddEdge (rightStNo, rightEnNo);
            if ( dest->hasBackData() ) {
                dest->ebData[nEdge].pathID=pathID;
                dest->ebData[nEdge].pieceID=pieceID;
                dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
            }
        }
    }
    else
    {
        /*		Geom::Point     biss;
                        biss=next.x-prev.x;
                        biss.y=next.y-prev.y;
                        double   c2=cross(biss, next);
                        double   l=width/c2;
                        double		projn=l*(dot(biss,next));
                        double		projp=-l*(dot(biss,prev));
                        if ( projp <= 0.5*prevL && projn <= 0.5*nextL ) {
                        double   x,y;
                        x=pos.x+l*biss.x;
                        y=pos.y+l*biss.y;
                        rightEnNo=rightStNo=dest->AddPoint(x,y);
                        } else {*/
        rightStNo = dest->AddPoint (pos - width*pnor);
        rightEnNo = dest->AddPoint (pos - width*nnor);
//        int midNo = dest->AddPoint (pos);
//        int nEdge=dest->AddEdge (rightStNo, midNo);
        int nEdge=dest->AddEdge (rightStNo, rightEnNo);
        if ( dest->hasBackData() ) {
            dest->ebData[nEdge].pathID=pathID;                                  
            dest->ebData[nEdge].pieceID=pieceID;
            dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
        }
//        nEdge=dest->AddEdge (midNo, rightEnNo);
//        if ( dest->hasBackData() ) {
//            dest->ebData[nEdge].pathID=pathID;
//            dest->ebData[nEdge].pieceID=pieceID;
//            dest->ebData[nEdge].tSt=dest->ebData[nEdge].tEn=tID;
//        }
        //              }
    }
}


// a very ugly way to produce round joins: doing one (or two, depend on the angle of the join) quadratic bezier curves
// but since most joins are going to be small, nobody will notice -- but somebody noticed and now the ugly stuff is gone! so:

// a very nice way to produce round joins, caps or dots
void Path::RecRound(Shape *dest, int sNo, int eNo, // start and end index
        Geom::Point const &iS, Geom::Point const &iE, // start and end point
        Geom::Point const &nS, Geom::Point const &nE, // start and end normal vector
        Geom::Point &origine, float width) // center and radius of round
{
    //Geom::Point diff = iS - iE;
    //double dist = dot(diff, diff);
    if (width < 0.5 || dot(iS - iE, iS - iE)/width < 2.0) {
        dest->AddEdge(sNo, eNo);
        return;
    }
    double ang, sia, lod;
    if (nS == -nE) {
        ang = M_PI;
        sia = 1;
    } else {
        double coa = dot(nS, nE);
        sia = cross(nE, nS);
        ang = acos(coa);
        if ( coa >= 1 ) {
            ang = 0;
        }
        if ( coa <= -1 ) {
            ang = M_PI;
        }
    }
    lod = 0.02 + 10 / (10 + width); // limit detail to about 2 degrees (180 * 0.02/Pi degrees)
    ang /= lod;

    int nbS = (int) floor(ang);
    Geom::Rotate omega(((sia > 0) ? -lod : lod));
    Geom::Point cur = iS - origine;
    //  StrokeNormalize(cur);
    //  cur*=width;
    int lastNo = sNo;
    for (int i = 0; i < nbS; i++) {
        cur = cur * omega;
        Geom::Point m = origine + cur;
        int mNo = dest->AddPoint(m);
        dest->AddEdge(lastNo, mNo);
        lastNo = mNo;
    }
    dest->AddEdge(lastNo, eNo);
}

namespace Util {

struct TreeifyResult
{
    std::vector<int> preorder; ///< The preorder traversal of the nodes, a permutation of {0, ..., N - 1}.
    std::vector<int> num_children; ///< For each node, the number of direct children.
};

/**
 * Given a collection of nodes 0 ... N - 1 and a containment function,
 * attempt to organise the nodes into a tree (or forest) such that
 * contains(i, j) is true precisely when i is an ancestor of j.
 */
TreeifyResult treeify(int N, std::function<bool(int, int)> const &contains)
{
    // Todo: (C++23) Refactor away to a recursive lambda.
    class Treeifier
    {
    public:
        Treeifier(int N, std::function<bool(int, int)> const &contains)
            : N{N}
            , contains{contains}
            , data(N)
        {
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    if (j != i && contains(i, j)) {
                        data[j].num_containers++;
                        data[i].contained.emplace_back(j);
                    }
                }
            }

            result.num_children.resize(N);

            for (int i = 0; i < N; i++) {
                if (data[i].num_containers == 0) {
                    visit(i);
                }
            }

            for (int i = 0; i < N; i++) {
                if (data[i].num_containers != -1) {
                    std::cerr << "Problem in treeify(): missed node" << std::endl;
                    result.preorder.emplace_back(i);
                }
            }

            assert(result.preorder.size() == N);
        }

        TreeifyResult moveResult() { return std::move(result); }

    private:
        // Input
        int N{};
        std::function<bool(int, int)> const &contains;

        // State
        struct Data
        {
            int num_containers = 0;
            std::vector<int> contained;
        };
        std::vector<Data> data;

        // Output
        TreeifyResult result;

        void visit(int i)
        {
            result.preorder.emplace_back(i);

            for (auto j : data[i].contained) {
                data[j].num_containers--;
            }

            for (auto j : data[i].contained) {
                if (data[j].num_containers == 0) {
                    result.num_children[i]++;
                    visit(j);
                }
            }

            data[i].num_containers = -1;
        }
    };

    return Treeifier(N, contains).moveResult();
}

} // namespace Util


/*
* Checks if the path describes a simple rectangle, returning true and populating rect if it does.
*/
Geom::OptRect check_simple_rect(Geom::PathVector const &pathv, double precision)
{
    // "Simple rectangle" is a single path with 4 line segments.
    if (pathv.size() != 1) {
        return {};
    }

    auto const &path = pathv.front();
    // might be a better way to check if all the curves are line segments
    if (!path.closed() || path.size() != 4 ||
        std::any_of(path.begin(), path.end(), [](auto const &curve) { return !curve.isLineSegment(); }))
    {
        return {};
    }

    auto const p0 = path.initialPoint();
    auto start = p0;
    int prev_change = 0;
    for (auto const &curve : path) {
        auto end = curve.finalPoint();
        // define a change in X as 1, change in Y as -1. Both is 0
        int const change = !Geom::are_near(start.x(), end.x(), precision) - !Geom::are_near(start.y(), end.y(), precision);
        if (change == 0 || change == prev_change) {
            // Changing in both x and y, or continuing the same direction as the previous segment.
            return {};
        }
        start = end;
        prev_change = change;
    }

    // if we've made it this far, we can assume that the final point of the second segment is the opposite corner
    auto const p1 = path[1].finalPoint();
    return Geom::Rect(p0, p1);
}

/**
 * Checks whether the filled region defined by pathvector @a a and fill rule @a fill_rule
 * completely contains pathvector @a b.
 */
bool pathv_fully_contains(Geom::PathVector const &a, Geom::PathVector const &b, FillRule fill_rule, double precision = Geom::EPSILON)
{
    // Fast-path the case where a is a rectangle.
    if (auto const a_rect = check_simple_rect(a, precision)) {
        return a_rect->contains(b.boundsExact());
    }

    // At minimum, bbox of a must contain bbox of b
    if (!a.boundsExact()->contains(b.boundsExact())) {
        return false;
    }

    // There must be no intersections of edges.
    // Fixme: This condition is too strict.
    if (!a.intersect(b, precision).empty()) {
        return false;
    }

    // Check winding number of first node of each subpath.
    // Fixme: This condition is also too strict.
    for (auto const &path : b) {
        if (!is_point_inside(fill_rule, a.winding(path.initialPoint()))) {
            return false;
        }
    }

    // BBox is fully contained, no intersections, passed the winding test, the path must be contained
    return true;
}

/// Attempts to find a point in the interior of a filled path.
static std::optional<Geom::Point> find_interior_point(Geom::Path const &path, FillRule fill_rule, std::default_random_engine &gen)
{
    auto ran = [&] { return std::uniform_real_distribution()(gen); };

    auto const bounds = path.boundsFast();
    if (!bounds) {
        return {};
    }

    constexpr int max_iterations = 10; // arbitrary cutoff, typically one iteration needed
    for (int i = 0; i < max_iterations; i++) {
        // Pick a random horizontal line through the path.
        auto const y = Geom::lerp(ran(), bounds->top(), bounds->bottom());
        auto const line = Geom::LineSegment{Geom::Point{bounds->left() - bounds->width(), y}, Geom::Point{bounds->right() + bounds->width(), y}};

        // Intersect the line with the path, recording the intersection times on the line.
        std::vector<double> times;
        for (auto const &curve : path) {
            for (auto const &intersection : line.intersect(curve)) {
                times.emplace_back(intersection.first);
            }
        }

        // Interior-test the points exactly halfway between intersections.
        for (int j = 1; j < times.size(); j++) {
            auto const t = (times[j - 1] + times[j]) / 2;
            auto const p = line.pointAt(t);
            if (is_point_inside(fill_rule, path.winding(p))) {
                return p;
            }
        }
    }

    return {};
}

/**
 * Given a pathvector @a pathv and fill rule @a fill_rule, compute pathv_fully_contains(pathv[i], pathv[j], fill_rule)
 * for all possible i != j. This class must be used with the Geom::Sweeper API to actually compute results. The results
 * are then available by calling contains(i, j);
 */
class PathContainmentSweeper
{
public:
    using ItemIterator = Geom::PathVector::const_iterator;

    PathContainmentSweeper(Geom::PathVector const &pathv, FillRule fill_rule, double precision = Geom::EPSILON)
        : _pathv{pathv}
        , _fill_rule{fill_rule}
        , _precision{precision}
        , _contains(_pathv.size() * _pathv.size(), false) // allocate space for two-dimensional array
    {}

    Geom::PathVector const &items() const { return _pathv; }

    Geom::Interval itemBounds(ItemIterator path) const
    {
        auto const r = path->boundsFast();
        return r ? (*r)[Geom::X] : Geom::Interval{};
    }

    void addActiveItem(ItemIterator incoming)
    {
        for (auto const &path : _active) {
            _checkPair(path, incoming);
            _checkPair(incoming, path);
        }
        _active.push_back(incoming);
    }

    void removeActiveItem(ItemIterator to_remove)
    {
        auto const it = std::find(_active.begin(), _active.end(), to_remove);
        std::swap(*it, _active.back());
        _active.pop_back();
    }

    //// Return the value of pathv_fully_contains(pathv[i], pathv[j], fill_rule).
    bool contains(int i, int j) const { return _contains[index(i, j)]; }

private:
    Geom::PathVector const &_pathv;
    FillRule const _fill_rule;
    double const _precision;

    std::vector<ItemIterator> _active;
    std::vector<bool> _contains;

    int index(int i, int j) const { return i * _pathv.size() + j; }

    void _checkPair(ItemIterator a, ItemIterator b)
    {
        if (pathv_fully_contains(*a, *b, _fill_rule)) {
            auto const ia = std::distance(_pathv.begin(), a);
            auto const ib = std::distance(_pathv.begin(), b);
            _contains[index(ia, ib)] = true;
        }
    }
};

/**
 * Given a pathvector and a tree structure on its subpaths representing how they are nested,
 * split the pathvector into its connected components when filled with the given fill rule.
 */
class PathContainmentTraverser
{
public:
    PathContainmentTraverser(Geom::PathVector &paths, Util::TreeifyResult const &tree, FillRule fill_rule)
        : _paths{paths}
        , _tree{tree}
        , _fill_rule{fill_rule}
    {
        while (_pos < tree.preorder.size()) {
            _visit(nullptr, 0, nullptr);
        }
    }

    std::vector<Geom::PathVector> &&moveResult() { return std::move(_result); }

private:
    // Input
    Geom::PathVector &_paths;
    Util::TreeifyResult const &_tree;
    FillRule const _fill_rule;

    // State
    std::default_random_engine _gen{std::random_device()()};
    int _pos = 0;

    // Output
    std::vector<Geom::PathVector> _result;

    void _visit(Geom::Path const *parent, int winding, Geom::PathVector *component)
    {
        // Visit the next node in the preorder traversal.
        int const x = _tree.preorder[_pos];
        _pos++;

        // Determine winding number at paths[x] of its parents in the tree.
        // Skip the computation for fill_justDont, as it would be unused.
        if (parent && _fill_rule != fill_justDont) {
            if (auto const p = find_interior_point(_paths[x], _fill_rule, _gen)) {
                winding += parent->winding(*p);
            }
        }

        // Determine if paths[x] is inside a hole. If so, we start a new component.
        bool const boundary = !is_point_inside(_fill_rule, winding);

        std::optional<Geom::PathVector> pathv;
        if (boundary) {
            pathv.emplace();
            component = &*pathv;
        }

        assert(component);
        component->push_back(std::move(_paths[x]));

        for (int i = 0; i < _tree.num_children[x]; i++) {
            _visit(&_paths[x], winding, component);
        }

        if (boundary) {
            _result.emplace_back(std::move(*pathv));
        }
    }
};

std::vector<Geom::PathVector> split_non_intersecting_paths(const Geom::PathVector &paths, FillRule fill_rule/* = FillRule::fill_nonZero*/)
{
    Geom::PathVector paths_copy {paths.begin(), paths.end()};
    auto path_containment = PathContainmentSweeper{paths_copy, FillRule::fill_nonZero};
    Geom::Sweeper{path_containment}.process();

    auto const tree = Util::treeify(paths_copy.size(), [&] (int i, int j) {
        return path_containment.contains(i, j);
    });

    return PathContainmentTraverser(paths_copy, tree, fill_rule).moveResult();
}

/*
  Local Variables:
  mode:c++
  c-file-style:"stroustrup"
  c-file-offsets:((innamespace . 0)(inline-open . 0)(case-label . +))
  indent-tabs-mode:nil
  fill-column:99
  End:
*/
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=8:softtabstop=4:fileencoding=utf-8:textwidth=99 :
