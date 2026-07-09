import { connect } from "@/dbConfig/dbConfig";
import { NextRequest, NextResponse } from "next/server";
import User from "@/models/userModel";

export async function POST(request: NextRequest) {
    try {
        await connect();
        const reqBody = await request.json();
        const { token } = reqBody;
        console.log(token);

        const user = await User.findOne({
            verifyToken: token,
            verifyTokenExpiry: { $gt: Date.now() },
        });

        if (!user) {
            return NextResponse.json({ error: "Invalid token" }, { status: 400 });
        }
        console.log(user);

        user.isVerified = true; // Fixed the typo: changed `isVerfied` to `isVerified`
        user.verifyToken = undefined;
        user.verifyTokenExpiry = undefined;
        await user.save();

        return NextResponse.json({
            message: "Email verified successfully",
            success: true,
        });

    } catch (error: unknown) {
        console.error("Error verifying email:", error);

        // Ensure proper error handling
        const errorMessage = error instanceof Error ? error.message : "An unknown error occurred";

        return NextResponse.json({ error: errorMessage }, { status: 500 });
    }
}
